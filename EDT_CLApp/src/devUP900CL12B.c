/***************************************************************************/
/* Filename: devUP900CL12B.c                                               */
/* Description: EPICS device support for UNIQVision UP900CL-12B camera     */
/***************************************************************************/

#define NO_STRDUP
#define NO_STRCASECMP
#define NO_MAIN

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <edtinc.h>
#include <libpdv.h>
#include <alarm.h>
#include <errno.h>
#include <dbCommon.h>
#include <dbDefs.h>
#include <recSup.h>
#include <recGbl.h>
#include <devSup.h>
#include <devLib.h>
#include <link.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <special.h>
#include <cvtTable.h>
#include <cantProceed.h>
#include <ellLib.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>

#include <aiRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <waveformRecord.h>
#include <stringoutRecord.h>
#include <epicsVersion.h>

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
#include <epicsExport.h>
#endif

/* Added this because of EPICS R3-14-12 */
#ifndef max
#define max(x, y)       (((x) < (y)) ? (y) : (x))
#endif
#ifndef min
#define min(x, y)       (((x) < (y)) ? (x) : (y))
#endif

#include "devCommonCameraLib.h"

int UP900CL12B_DEV_DEBUG = 1;

/* some constants about UNIQVision UP900CL-12B camera */

#define CAMERA_MODEL_NAME "UP900CL-12B"
#define CAMERA_CONFIG_NAME "up900cl12b.cfg"

#if 0	/* We don't hardcode here, since cfg file might use different */
#define	NUM_OF_COL	1392
#define	NUM_OF_ROW	1040
#endif

/* This is always no change, so we hardcode to save trouble of type of pointer */
#define	NUM_OF_BITS	12

#define SIZE_OF_PIXEL	0.00465	/* millimeter per pixel */

/* some constants about UNIQVision UP900CL-12B camera */

#define	NUM_OF_FRAMES	100	/* number of frames in circular buffer */

#define IMAGE_TS_EVT_NUM 53	/* upon which event we timestamp image */

#ifdef vxWorks
#define CAMERA_THREAD_PRIORITY	(10)
#else
/*#define CAMERA_THREAD_PRIORITY (epicsThreadPriorityMedium)*/
#define CAMERA_THREAD_PRIORITY (epicsThreadPriorityHigh - 1)
#endif
#define CAMERA_THREAD_STACK	(0x20000)

/* image data structure */
typedef struct IMAGE_BUF
{
    epicsTimeStamp      timeStamp;
    epicsTimeStamp      triggerTimeStamp;

    unsigned short int	*pImage;	/* UP900CL12B is 12-bit camera */

    unsigned short int	**ppRow;	/* pointers to each row */

    unsigned int	*pProjectionX;	/* projection to X, sum of every column */
    unsigned int	*pProjectionY;	/* projection to Y, sum of every row */

    double		fwhmX;		/* The unit is millimeter */
    double		fwhmY;		/* The unit is millimeter */
    double		centroidX;	/* The unit is millimeter */
    double		centroidY;	/* The unit is millimeter */
    /* We might add ROI info here when we support HW ROI */
} IMAGE_BUF;

char *SHARED_HISTORY_BLOCK = 0; /* Holds history buffer for ALL cameras */

/* UP900CL-12B operation data structure defination */
/* The first fourteen elements must be same cross all types of cameras, same of COMMON_CAMERA */
typedef struct UP900CL12B_CAMERA
{
    ELLNODE		node;		/* link list node */

    char		*pCameraName;	/* name of this camera, must be unique */

    unsigned int	unit;		/* index of EDT DV C-LINK PMC card */
    unsigned int	channel;	/* channel on  EDT DV C-LINK PMC card */

    char		*pModelName;	/* model name of camera */
    char		*pConfigName;	/* configuration name for camera */

    PdvDev		*pCameraHandle;	/* handle of PdvDev */

    int			numOfCol;	/* number of column of this camera */
    int			numOfRow;	/* number of row of this camera */
    int			numOfBits;	/* number of bits of this camera */

    int			imageSize;	/* image size in byte */
    int			dmaSize;	/* dma size of image in byte, usually same as imageSize */

    CAMERASTARTFUNC	pCameraStartFunc;
    /* No change above, these elements must be identical cross all cameras */
    int			cameraMode;     /* reserved for future, could be free run, pulse width or more */

    int			saveImage;	/* data path, circular buffer (1) or ping-pong buffer (0) */

    /* Software ROI */
    int			startPixelX;	/* (x,y) for the up left pixel of sub-image */
    int			startPixelY;	/* (x,y) for the up left pixel of sub-image */
    int			nPixelsX;	/* number of horizontal pixels for the up left pixel of sub-image */
    int			nPixelsY;	/* number of vertical pixels for the up left pixel of sub-image */
    /* Software ROI */

    int			frameCounts;	/* debug information to show trigger frequency */

    IMAGE_BUF		pingpongBuf[2];	/* ping-pong image buffer */
    int			pingpongFlag;	/* ping-pong buffer flag, indicate which buffer can be written */

    char		* phistoryBlock;	/* The image history buffer need big memory, we just malloc a big block */
  IMAGE_BUF		historyBuf[NUM_OF_FRAMES];
    unsigned int	historyBufIndex;	/* Indicate which history buffer is ready to be written */
    unsigned int	historyBufFull;
    epicsMutexId	historyBufMutexLock;	/* history buffer mutex semaphore */

    signed int		historyBufReadOffset;	/* The offset from the latest frame, starts from 0, must be 0 or negative number */

    epicsMutexId	mutexLock;	/*  general mutex semaphore */
    char                saveImageDir[100];
    char                imageName[100];
    int                 triggerDAQ; /* Set by the TRIGGER_DAQ PV when starting acquisition for saving to disk */

    int                 numImagesDAQ; /* Number of images to be saved to disk (max is NUM_OF_FRAMES) */
    int                 statusDAQ; /* Current DAQ status */
    IOSCANPVT           ioscanpvt;
  uint32_t            readTimeMax; /** Max time taken to read image from camera */
  uint32_t            readTimeMin; /** Min time taken to read image from camera */
  float            saveTimeMax; /** Max time taken to save images to files */
  float            saveTimeMin; /** Min time taken to save images to files */
  unsigned int     imageTimestampEvent; /** Event whose timestamp is used to tag the images */

  struct timeval imageTS; /* time when image got read out from camera */
  struct timeval triggerTS; /* time when camera trigger was received */
  int numTriggersDAQ; /* Number of triggers received during DAQ */
  epicsUInt32 maxImageDelayUs; /** Max delay (in usec) between the trigger and image readout */
  epicsUInt32 minImageDelayUs; /** Min delay (in usec) between the trigger and image readout */
  epicsUInt32 avgImageDelayUs; /** Avg delay (in usec) between the trigger and image readout */
  int mismatchIdCount; /** Counts if pulse ID between trigger and image readout disagree - which means read out took too long */
  int numImagesDAQCnt; /** Number of images taken in current DAQ */

} UP900CL12B_CAMERA;

static int image12b_noise_reduce(unsigned char * image, int image_size, float threshold_ratio);
static int image12b_projection_calc(const unsigned char * image, int * proj_H, int num_col, int * proj_V, int num_row);
static int image12b_centroid_calc(int * proj_H, int num_col, int * proj_V, int num_row, double * cen_H, double * cen_V);

#define DAQ_READY 0
#define DAQ_ACQUIRING_IMAGES 1
#define DAQ_SAVING_IMAGES 2

unsigned int UP900_SHIFT_4BITS = 0;

static char IMAGE_DIRECTORY[1024];


#define LOWER_17_BIT_MASK       (0x0001FFFF)    /* (2^17)-1            */
#define PULSEID(time)           ((time).nsec & LOWER_17_BIT_MASK)

static uint64_t cg64()
{
struct timespec now;
	clock_gettime( CLOCK_MONOTONIC, &now );
	return  (uint64_t)now.tv_sec * (uint64_t)1000000 +  (uint64_t)now.tv_nsec/(uint64_t)1000;
}

static int UP900CL12B_FlushBufferToDisk(int num_images, UP900CL12B_CAMERA * pCamera)
{
  time_t now;
  struct tm *timeinfo;
  char timestr[30];
  size_t size;
  uint64_t td;
  uint32_t rval;

  time(&now);
  timeinfo = localtime(&now);
  strftime(timestr, 30, "%m-%d-%Y-%H-%M-%S", timeinfo);
/*   printf("[%s] Saving collected images to disk (%s).\n", timestr, pCamera->saveImageDir); */

  int images = pCamera->historyBufIndex;
  char file_name[300];

  if (images == 0) {
    printf("No images were recorded, can't save files.\n");
    return -1;
  }

  time_t a, b;
  time_t t = time(0);
/*   printf("Start Time: %s", ctime(&t)); */

  a = time(0);
  int image_index = 0;
  int image_size = pCamera->numOfCol * pCamera->numOfRow * sizeof(unsigned short int);

  td = cg64();
  
#define SINGLE_FILE 1
  
  FILE *image_file;

  /** SINGLE FILE **/
#ifdef SINGLE_FILE
/*   sprintf(file_name, "%s/%s-%d-%d-%d.pgm", */
/* 	  pCamera->saveImageDir, */
/* 	  pCamera->pCameraName, */
/* 	  pCamera->historyBuf[0].timeStamp.secPastEpoch, */
/* 	  pCamera->historyBuf[0].timeStamp.nsec, */
/* 	  PULSEID(pCamera->historyBuf[image_index].timeStamp)); */

  sprintf(file_name, "%s/%s-%s.images",
	  pCamera->saveImageDir,
	  pCamera->imageName,
	  timestr);
  
  image_file = fopen(file_name, "w");
  if (image_file == NULL) {
    printf("Error opening file %s (errno=%d)\n", file_name, errno);
    return -1;
  }

  FILE *header_file;
  sprintf(file_name, "%s/%s-%s.header",
	  pCamera->saveImageDir,
	  pCamera->imageName,
	  timestr);
  
  header_file = fopen(file_name, "w");
  if (header_file == NULL) {
    printf("Error opening file %s\n", file_name);
    return -1;
  }
  
#endif

  
  for (; image_index < num_images; image_index++) {
#ifndef SINGLE_FILE
    sprintf(file_name, "%s/%s-%d-%d-%d.pgm",
	    pCamera->saveImageDir,
 	    pCamera->pCameraName,
	    pCamera->historyBuf[image_index].timeStamp.secPastEpoch,
	    pCamera->historyBuf[image_index].timeStamp.nsec,
	    PULSEID(pCamera->historyBuf[image_index].timeStamp));

    if (image_index > 0) {
      if ((pCamera->historyBuf[image_index].timeStamp.nsec == 
	   pCamera->historyBuf[image_index - 1].timeStamp.nsec) &&
	  (pCamera->historyBuf[image_index].timeStamp.secPastEpoch == 
	   pCamera->historyBuf[image_index - 1].timeStamp.secPastEpoch)) {
	sprintf(file_name, "%s/%s-%d-%d-%d.2.pgm",
		pCamera->saveImageDir,
		pCamera->pCameraName,
		pCamera->historyBuf[image_index].timeStamp.secPastEpoch,
		pCamera->historyBuf[image_index].timeStamp.nsec,
		PULSEID(pCamera->historyBuf[image_index].timeStamp));
      }
    }

    image_file = fopen(file_name, "w");
    if (image_file == NULL) {
      printf("Error opening file %s\n", file_name);
      return -1;
    }
#endif

    now = pCamera->historyBuf[image_index].timeStamp.secPastEpoch;
    timeinfo = localtime(&now);
    strftime(timestr, 30, "%m/%d %H:%M:%S", timeinfo);

#ifdef SINGLE_FILE
    fprintf(header_file, "P5\n");
    fprintf(header_file, "# Camera: %s\n", pCamera->pCameraName);
    fprintf(header_file, "# Date: %s\n", timestr );
    fprintf(header_file, "# EVR timestamp: %d sec %d nsec\n",
	    pCamera->historyBuf[image_index].timeStamp.secPastEpoch,
	    pCamera->historyBuf[image_index].timeStamp.nsec);
    fprintf(header_file, "# PULSEID: %d\n",
	    PULSEID(pCamera->historyBuf[image_index].timeStamp));
    fprintf(header_file, "# Sequence #%d\n", image_index);
    fprintf(header_file, "%d %d\n", pCamera->numOfCol, pCamera->numOfRow);
    fprintf(header_file, "4096\n");
#else
    fprintf(image_file, "P5\n");
    fprintf(image_file, "# Camera: %s\n", pCamera->pCameraName);
    fprintf(image_file, "# Date: %s\n", timestr );
    fprintf(image_file, "# EVR timestamp: %d sec %d nsec\n",
	    pCamera->historyBuf[image_index].timeStamp.secPastEpoch,
	    pCamera->historyBuf[image_index].timeStamp.nsec);
    fprintf(header_file, "# PULSEID: %d\n",
	    PULSEID(pCamera->historyBuf[image_index].timeStamp));
    fprintf(image_file, "# Sequence #%d\n", image_index);
    fprintf(image_file, "%d %d\n", pCamera->numOfCol, pCamera->numOfRow);
    fprintf(image_file, "4096\n");
#endif

/* #ifdef SINGLE_FILE */
/*     fprintf(header_file, "%d\n", PULSEID(pCamera->historyBuf[image_index].timeStamp)); */
/* #endif  */
    
    size = fwrite(pCamera->historyBuf[image_index].pImage, image_size, 1, image_file);

#ifndef SINGLE_FILE
    fclose(image_file);
#endif
  }
#ifdef SINGLE_FILE
  fclose(image_file);
  fclose(header_file);
#endif
  td = cg64() - td;
  rval = (uint32_t)td;

  b = time(0);

/*   time(&now); */
/*   timeinfo = localtime(&now); */
/*   strftime(timestr, 30, "%m/%d %H:%M:%S", timeinfo); */
/*   printf("[%s] Done saving collected images to disk (%s).\n", timestr, pCamera->saveImageDir); */

  float rate = image_size * num_images;
  if (b - a == 0) {
    printf("ERROR: Apparently no files were saved, please check configuration.\n");
    return -1;
  }
  rate /= (b - a);
  rate /= 1024;
  rate /= 1024;
  printf("Rate: %f MB/s; Time: %d sec; %d usec\n", rate, b - a, rval);
  
  return 0;
}

static epicsUInt32 us_since_last_trigger(UP900CL12B_CAMERA *pCamera) {
  struct timeval now;

  gettimeofday(&now, 0);

  if (now.tv_usec < pCamera->triggerTS.tv_usec) {
    now.tv_usec += 1000000;
    now.tv_sec--;
  }
  now.tv_usec  = now.tv_usec - pCamera->triggerTS.tv_usec;
  now.tv_usec += now.tv_sec  - pCamera->triggerTS.tv_sec;

  return (epicsUInt32) now.tv_usec;
}

static int UP900CL12B_Poll(UP900CL12B_CAMERA * pCamera)
{
    int loop, saveImage;
    int oldSaveImage = 0;
    unsigned short int *pNewFrame;
    IMAGE_BUF * pImageBuf;
    uint64_t beginTime;
    uint64_t endTime;
    uint32_t readTime;
    int saveFiles = 0;

    if(pCamera == NULL)
    {
        errlogPrintf("Camera polling thread quits because no legal pCamera!\n");
        return -1;
    }

    while(TRUE)
    {
        /* waiting for new frame */
        pNewFrame = (unsigned short int *)pdv_wait_image(pCamera->pCameraHandle);
	saveFiles = 0;
	beginTime = cg64();

        /* Got a new frame */
        pCamera->frameCounts++;

	epicsMutexLock(pCamera->historyBufMutexLock);

	saveImage = 0;
	oldSaveImage = pCamera->saveImage;
	if (pCamera->statusDAQ == DAQ_ACQUIRING_IMAGES) {
	  gettimeofday(&(pCamera->imageTS), 0);
	  saveImage = 1;
	}

        if(saveImage || oldSaveImage)
        {/* New frame goes into history buffer */
            pImageBuf = pCamera->historyBuf + pCamera->historyBufIndex;
        }
        else
        {/* New frame goes into ping-pong buffer */
            pImageBuf = pCamera->pingpongBuf + pCamera->pingpongFlag;
        }

        /* Set time stamp even data is not usable */
/* 	epicsTimeGetEvent(&(pImageBuf->timeStamp), IMAGE_TS_EVT_NUM); */
	epicsTimeGetEvent(&(pImageBuf->timeStamp), pCamera->imageTimestampEvent);

        memcpy((void*)(pImageBuf->pImage), (void*)pNewFrame, pCamera->imageSize);

	endTime = cg64();
	readTime = (uint32_t)(endTime - beginTime);
	beginTime = endTime;
	if (readTime > pCamera->readTimeMax) {
	  pCamera->readTimeMax = readTime;
	}
	if (readTime < pCamera->readTimeMin) {
	  pCamera->readTimeMin = readTime;
	}

        if(UP900_SHIFT_4BITS > 0)
        {
            for(loop=0; loop<pCamera->numOfCol * pCamera->numOfRow; loop++) pImageBuf->pImage[loop] >>= 4;
        }

        /* Calculate projiection, FWHM, centroid ... */

	if (pCamera->statusDAQ == DAQ_ACQUIRING_IMAGES) 
        {/* New frame goes into history buffer */

	  epicsUInt32 this_time = us_since_last_trigger(pCamera);

	  if (this_time > pCamera->maxImageDelayUs) {
	    pCamera->maxImageDelayUs = this_time;
	  }
	  if (this_time < pCamera->minImageDelayUs) {
	    pCamera->minImageDelayUs = this_time;
	  }
	  
	  /* Running average: fn+1 = 127/128 * fn + 1/128 * x = fn - 1/128 fn + 1/128 x */
	  pCamera->avgImageDelayUs += ((int)(- pCamera->avgImageDelayUs + this_time)) >> 8;
	  /*
	  printf("max=%d min=%d avg=%d\n",
		 pCamera->maxImageDelayUs, pCamera->minImageDelayUs, pCamera->avgImageDelayUs);
	  */
	  if (PULSEID(pImageBuf->timeStamp) != PULSEID(pImageBuf->triggerTimeStamp)) {
	    pCamera->mismatchIdCount++;
	    /*
	    printf("img %d, trig %d\n", 
		   PULSEID(pImageBuf->timeStamp),
		   PULSEID(pImageBuf->triggerTimeStamp));
	    */
	  }

	  pCamera->historyBufIndex++;
	  pCamera->numImagesDAQCnt++;  
	  if(pCamera->historyBufIndex >= pCamera->numImagesDAQ) {
	    saveFiles = 1;
	    pCamera->historyBufFull = 1;
	    pCamera->statusDAQ = DAQ_SAVING_IMAGES;
	    scanIoRequest(pCamera->ioscanpvt);
	    UP900CL12B_FlushBufferToDisk(pCamera->numImagesDAQ, pCamera);
	    pCamera->historyBufIndex = 0;
	    pCamera->historyBufFull = 0; /* "Clear" buffer for next DAQ cycle */
	    pCamera->triggerDAQ = 0;
	    pCamera->statusDAQ = DAQ_READY;
	    scanIoRequest(pCamera->ioscanpvt);
	  }
        }
        else if(oldSaveImage)
        {/* New frame goes into history buffer */
            pCamera->historyBufIndex++;
            if(pCamera->historyBufIndex >= NUM_OF_FRAMES)
            {
                pCamera->historyBufIndex = 0;
                pCamera->historyBufFull = 1;
            }
        }
	else
        {/* New frame goes into ping-pong buffer */
            pCamera->pingpongFlag = 1 - pCamera->pingpongFlag;
        }
	epicsMutexUnlock(pCamera->historyBufMutexLock);

	endTime = cg64();
	readTime = (uint32_t)(endTime - beginTime);
	if (saveFiles) {
	  float saveTime = readTime / 1000000;
	  if (saveTime > pCamera->saveTimeMax) {
	    pCamera->saveTimeMax = saveTime;
	  }
	  if (saveTime < pCamera->saveTimeMin) {
	    pCamera->saveTimeMin = saveTime;
	  }
	}
	else {
	}
    }
    return 0;
}

static int UP900CL12B_Start(UP900CL12B_CAMERA * pCamera)
{
    if(!pCamera)
    {
        errlogPrintf("UP900CL12B_Start is called with pCamera=NULL!\n");
        return -1;
    }

    /* In this application, we use UP900CL-12B async pulse width mode */
    pdv_enable_external_trigger(pCamera->pCameraHandle, PDV_PHOTO_TRIGGER);
    pdv_start_images(pCamera->pCameraHandle, 0);

    /* Create thread */
#ifdef vxWorks
    taskSpawn(pCamera->pCameraName, CAMERA_THREAD_PRIORITY, VX_FP_TASK, CAMERA_THREAD_STACK, (FUNCPTR)UP900CL12B_Poll, (int)pCamera, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#else
    epicsThreadMustCreate(pCamera->pCameraName, CAMERA_THREAD_PRIORITY, CAMERA_THREAD_STACK, (EPICSTHREADFUNC)UP900CL12B_Poll, (void *)pCamera);
#endif

    return 0;
}

int UP900CL12B_Init(char * name, int unit, int channel)
{
    int status, loop, looprow;

    UP900CL12B_CAMERA * pCamera = (UP900CL12B_CAMERA *) callocMustSucceed( 1, sizeof(UP900CL12B_CAMERA), "Allocate memory for UP900CL12B_CAMERA" );
    /* no bzero needed */

    /* common camera initialization */
    status = commonCameraInit(name, unit, channel, CAMERA_MODEL_NAME, CAMERA_CONFIG_NAME, (CAMERASTARTFUNC)UP900CL12B_Start, (CAMERA_ID)pCamera);
    if(status || pCamera->numOfBits != NUM_OF_BITS)
    {
        errlogPrintf("commonCameraInit failed for camera %s!\n", name);
        epicsThreadSuspendSelf();
        return -1;
    }

    /* pCamera->saveImage, default to ping-pong buffer (0) */

    pCamera->startPixelX = 0;
    pCamera->startPixelY = 0;
    pCamera->nPixelsX = pCamera->numOfCol;
    pCamera->nPixelsY = pCamera->numOfRow;

    pCamera->frameCounts = 0;

    /* Initialize ping-pong buffer */
    for(loop=0; loop<2; loop++)
    {
        bzero((char *)&(pCamera->pingpongBuf[loop].timeStamp), sizeof(epicsTimeStamp));

        pCamera->pingpongBuf[loop].pImage = (unsigned short int *)callocMustSucceed(1, pCamera->imageSize, "Allocate ping-pong buf");
        pCamera->pingpongBuf[loop].ppRow = (unsigned short int **)callocMustSucceed(pCamera->numOfRow, sizeof(unsigned short int *), "Allocate buf for row pointers");
        for(looprow=0; looprow<pCamera->numOfRow; looprow++)
            pCamera->pingpongBuf[loop].ppRow[looprow] = pCamera->pingpongBuf[loop].pImage + looprow * pCamera->numOfCol;

        pCamera->pingpongBuf[loop].pProjectionX = (unsigned int *)callocMustSucceed(pCamera->numOfCol, sizeof(unsigned int), "Allocate buf for Projection X");
        pCamera->pingpongBuf[loop].pProjectionY = (unsigned int *)callocMustSucceed(pCamera->numOfRow, sizeof(unsigned int), "Allocate buf for Projection Y");

        pCamera->pingpongBuf[loop].fwhmX = 0.0;
        pCamera->pingpongBuf[loop].fwhmY = 0.0;
        pCamera->pingpongBuf[loop].centroidX = 0.0;
        pCamera->pingpongBuf[loop].centroidY = 0.0;
    }
    pCamera->pingpongFlag = 0;

    /* Initialize history buffer */
    if(UP900CL12B_DEV_DEBUG) {
      printf("calloc %fMB memory\n", NUM_OF_FRAMES * pCamera->imageSize/1.0e6);
    }

    if (SHARED_HISTORY_BLOCK == 0) {
      SHARED_HISTORY_BLOCK = (char *)callocMustSucceed(NUM_OF_FRAMES,
						       pCamera->imageSize,
						       "Allocate huge his buf (ONCE)");
    }
    pCamera->phistoryBlock = SHARED_HISTORY_BLOCK;

/*     pCamera->phistoryBlock = (char *)callocMustSucceed(NUM_OF_FRAMES, */
/* 						       pCamera->imageSize, */
/* 						       "Allocate huge his buf"); */
    for(loop=0; loop<NUM_OF_FRAMES; loop++)
    {
        bzero((char *)&(pCamera->historyBuf[loop].timeStamp), sizeof(epicsTimeStamp));

        pCamera->historyBuf[loop].pImage = (unsigned short int *)(pCamera->phistoryBlock + loop * pCamera->imageSize);
        pCamera->historyBuf[loop].ppRow = (unsigned short int **)callocMustSucceed(pCamera->numOfRow, sizeof(unsigned short int *), "Allocate buf for row pointers");
        for(looprow=0; looprow<pCamera->numOfRow; looprow++)
            pCamera->historyBuf[loop].ppRow[looprow] = pCamera->historyBuf[loop].pImage + looprow * pCamera->numOfCol;

        pCamera->historyBuf[loop].pProjectionX = (unsigned int *)callocMustSucceed(pCamera->numOfCol, sizeof(unsigned int), "Allocate buf for Projection X");
        pCamera->historyBuf[loop].pProjectionY = (unsigned int *)callocMustSucceed(pCamera->numOfRow, sizeof(unsigned int), "Allocate buf for Projection Y");

        pCamera->historyBuf[loop].fwhmX = 0.0;
        pCamera->historyBuf[loop].fwhmY = 0.0;
        pCamera->historyBuf[loop].centroidX = 0.0;
        pCamera->historyBuf[loop].centroidY = 0.0;
    }
    pCamera->historyBufIndex = 0;
    pCamera->historyBufFull = 0;
    pCamera->historyBufMutexLock = epicsMutexMustCreate();

    pCamera->historyBufReadOffset = 0;

    pCamera->mutexLock = epicsMutexMustCreate();

    pCamera->statusDAQ = DAQ_READY;

    pCamera->readTimeMax = 0;
    pCamera->readTimeMin = 0xFFFFFFF;

    pCamera->saveTimeMax = 0;
    pCamera->saveTimeMin = 60 * 60;

    pCamera->numTriggersDAQ = 0;
    pCamera->maxImageDelayUs = 0;
    pCamera->minImageDelayUs = 0xFFFFFF;
    pCamera->avgImageDelayUs = 0;
    pCamera->mismatchIdCount = 0;
    pCamera->numImagesDAQCnt = 0;

    scanIoInit(&(pCamera->ioscanpvt));

    /* We successfully allocate all resource */
    return 0;
}

/* Device support implementation */

static long init_ai(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);
static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);
static long init_li(struct longinRecord *pli);
static long get_li_ioinfo(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt);
static long read_li(struct longinRecord *pli);
static long init_lo(struct longoutRecord *plo);
static long write_lo(struct longoutRecord *plo);
static long init_wf(struct waveformRecord *pwf);
static long read_wf(struct waveformRecord *pwf);
static long init_so(struct stringoutRecord *pso);
static long write_so(struct stringoutRecord *pso);


/* global struct for devSup */
typedef struct {
    long		number;
    DEVSUPFUN	report;
    DEVSUPFUN	init;
    DEVSUPFUN	init_record;
    DEVSUPFUN	get_ioint_info;
    DEVSUPFUN	read_write;
    DEVSUPFUN	special_linconv;
} UP900CL12B_DEV_SUP_SET;

UP900CL12B_DEV_SUP_SET devAiEDTCL_UP900_12B=   {6, NULL, NULL, init_ai,  NULL, read_ai,  NULL};
UP900CL12B_DEV_SUP_SET devBoEDTCL_UP900_12B=   {6, NULL, NULL, init_bo,  NULL, write_bo,  NULL};
UP900CL12B_DEV_SUP_SET devLiEDTCL_UP900_12B=   {6, NULL, NULL, init_li,  get_li_ioinfo, read_li,  NULL};
UP900CL12B_DEV_SUP_SET devLoEDTCL_UP900_12B=   {6, NULL, NULL, init_lo,  NULL, write_lo,  NULL};
UP900CL12B_DEV_SUP_SET devWfEDTCL_UP900_12B=   {6, NULL, NULL, init_wf,  NULL, read_wf,  NULL};
UP900CL12B_DEV_SUP_SET devSoEDTCL_UP900_12B=   {6, NULL, NULL, init_so,  NULL, write_so,  NULL};

#if	EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devAiEDTCL_UP900_12B);
epicsExportAddress(dset, devBoEDTCL_UP900_12B);
epicsExportAddress(dset, devLiEDTCL_UP900_12B);
epicsExportAddress(dset, devLoEDTCL_UP900_12B);
epicsExportAddress(dset, devWfEDTCL_UP900_12B);
epicsExportAddress(dset, devSoEDTCL_UP900_12B); 
#endif

typedef enum
{
    EPICS_RTYPE_NONE,
    EPICS_RTYPE_AI,
    EPICS_RTYPE_AO,
    EPICS_RTYPE_BI,
    EPICS_RTYPE_BO,
    EPICS_RTYPE_LI,
    EPICS_RTYPE_LO,
    EPICS_RTYPE_MBBI,
    EPICS_RTYPE_MBBO,
    EPICS_RTYPE_MBBID,
    EPICS_RTYPE_MBBOD,
    EPICS_RTYPE_SI,
    EPICS_RTYPE_SO,
    EPICS_RTYPE_WF
}   E_EPICS_RTYPE;

typedef enum {
    UP900CL12B_AI_CurFwhmX,
    UP900CL12B_AI_CurFwhmY,
    UP900CL12B_AI_CurCtrdX,
    UP900CL12B_AI_CurCtrdY,
    UP900CL12B_AI_HisFwhmX,
    UP900CL12B_AI_HisFwhmY,
    UP900CL12B_AI_HisCtrdX,
    UP900CL12B_AI_HisCtrdY,
    UP900CL12B_BO_SaveImage,
    UP900CL12B_BO_DAQReadReset,
    UP900CL12B_LI_NumOfCol,
    UP900CL12B_LI_NumOfRow,
    UP900CL12B_LI_NumOfBits,
    UP900CL12B_LI_FrameRate,
    UP900CL12B_LI_StartPixelX,
    UP900CL12B_LI_StartPixelY,
    UP900CL12B_LI_NumPixelsX,
    UP900CL12B_LI_NumPixelsY,
    UP900CL12B_LO_HisIndex,
    UP900CL12B_LO_StartPixelX,
    UP900CL12B_LO_StartPixelY,
    UP900CL12B_LO_NumPixelsX,
    UP900CL12B_LO_NumPixelsY,
    UP900CL12B_WF_CurImage,
    UP900CL12B_WF_HisImage,
    UP900CL12B_WF_CurProjX,
    UP900CL12B_WF_CurProjY,
    UP900CL12B_WF_HisProjX,
    UP900CL12B_WF_HisProjY,
    UP900CL12B_SO_SaveImageDir,
    UP900CL12B_SO_ImageName,
    UP900CL12B_BO_TriggerDAQ,
    UP900CL12B_BO_TriggerTS,
    UP900CL12B_LO_NumImagesDAQ,
    UP900CL12B_LI_StatusDAQ,
    UP900CL12B_LI_DAQDupId,
    UP900CL12B_LI_DAQReadMax,
    UP900CL12B_LI_DAQReadMin,
    UP900CL12B_LI_DAQReadAvg,
    UP900CL12B_LI_DAQImgCnt,
    UP900CL12B_LI_ReadMax,
    UP900CL12B_LI_ReadMin,
    UP900CL12B_AI_SaveMax,
    UP900CL12B_AI_SaveMin,
    UP900CL12B_LO_ImageTimestampEvent,
} E_UP900CL12B_FUNC;

static struct PARAM_MAP
{
        char param[40];
        E_EPICS_RTYPE rtype;;
        E_UP900CL12B_FUNC funcflag;
} param_map[] = {
    {"CurFwhmX",	EPICS_RTYPE_AI,	UP900CL12B_AI_CurFwhmX},
    {"CurFwhmY",	EPICS_RTYPE_AI,	UP900CL12B_AI_CurFwhmY},
    {"CurCtrdX",	EPICS_RTYPE_AI,	UP900CL12B_AI_CurCtrdX},
    {"CurCtrdY",	EPICS_RTYPE_AI,	UP900CL12B_AI_CurCtrdY},
    {"HisFwhmX",	EPICS_RTYPE_AI,	UP900CL12B_AI_HisFwhmX},
    {"HisFwhmY",	EPICS_RTYPE_AI,	UP900CL12B_AI_HisFwhmY},
    {"HisCtrdX",	EPICS_RTYPE_AI,	UP900CL12B_AI_HisCtrdX},
    {"HisCtrdY",	EPICS_RTYPE_AI,	UP900CL12B_AI_HisCtrdY},
    {"SaveImage",	EPICS_RTYPE_BO,	UP900CL12B_BO_SaveImage},
    {"DAQReadReset",	EPICS_RTYPE_BO,	UP900CL12B_BO_DAQReadReset},
    {"NumOfCol",	EPICS_RTYPE_LI,	UP900CL12B_LI_NumOfCol},
    {"NumOfRow",	EPICS_RTYPE_LI,	UP900CL12B_LI_NumOfRow},
    {"NumOfBits",	EPICS_RTYPE_LI,	UP900CL12B_LI_NumOfBits},
    {"FrameRate",	EPICS_RTYPE_LI,	UP900CL12B_LI_FrameRate},
    {"StartPixelX",	EPICS_RTYPE_LI,	UP900CL12B_LI_StartPixelX},
    {"StartPixelY",	EPICS_RTYPE_LI,	UP900CL12B_LI_StartPixelY},
    {"NumPixelsX",	EPICS_RTYPE_LI,	UP900CL12B_LI_NumPixelsX},
    {"NumPixelsY",	EPICS_RTYPE_LI,	UP900CL12B_LI_NumPixelsY},
    {"HisIndex",	EPICS_RTYPE_LO,	UP900CL12B_LO_HisIndex},
    {"StartPixelX",	EPICS_RTYPE_LO,	UP900CL12B_LO_StartPixelX},
    {"StartPixelY",	EPICS_RTYPE_LO,	UP900CL12B_LO_StartPixelY},
    {"NumPixelsX",	EPICS_RTYPE_LO,	UP900CL12B_LO_NumPixelsX},
    {"NumPixelsY",	EPICS_RTYPE_LO,	UP900CL12B_LO_NumPixelsY},
    {"CurImage",	EPICS_RTYPE_WF,	UP900CL12B_WF_CurImage},
    {"HisImage",	EPICS_RTYPE_WF,	UP900CL12B_WF_HisImage},
    {"CurProjX",	EPICS_RTYPE_WF,	UP900CL12B_WF_CurProjX},
    {"CurProjY",	EPICS_RTYPE_WF,	UP900CL12B_WF_CurProjY},
    {"HisProjX",	EPICS_RTYPE_WF,	UP900CL12B_WF_HisProjX},
    {"HisProjY",	EPICS_RTYPE_WF,	UP900CL12B_WF_HisProjY},
    {"SaveImageDir",    EPICS_RTYPE_SO, UP900CL12B_SO_SaveImageDir},
    {"ImageName",       EPICS_RTYPE_SO, UP900CL12B_SO_ImageName},
    {"TriggerDAQ",      EPICS_RTYPE_BO, UP900CL12B_BO_TriggerDAQ},
    {"TriggerTS",       EPICS_RTYPE_BO, UP900CL12B_BO_TriggerTS},
    {"NumImagesDAQ",    EPICS_RTYPE_LO, UP900CL12B_LO_NumImagesDAQ},
    {"StatusDAQ",       EPICS_RTYPE_LI, UP900CL12B_LI_StatusDAQ},
    {"DAQDupId",         EPICS_RTYPE_LI, UP900CL12B_LI_DAQDupId},
    {"DAQReadMax",         EPICS_RTYPE_LI, UP900CL12B_LI_DAQReadMax},
    {"DAQReadMin",         EPICS_RTYPE_LI, UP900CL12B_LI_DAQReadMin},
    {"DAQReadAvg",         EPICS_RTYPE_LI, UP900CL12B_LI_DAQReadAvg},
    {"DAQImgCnt",         EPICS_RTYPE_LI, UP900CL12B_LI_DAQImgCnt},
    {"ReadMax",         EPICS_RTYPE_LI, UP900CL12B_LI_ReadMax},
    {"ReadMin",         EPICS_RTYPE_LI, UP900CL12B_LI_ReadMin},
    {"SaveMax",         EPICS_RTYPE_AI, UP900CL12B_AI_SaveMax},
    {"SaveMin",         EPICS_RTYPE_AI, UP900CL12B_AI_SaveMin},
    {"ImageTimestampEvent", EPICS_RTYPE_LO, UP900CL12B_LO_ImageTimestampEvent}
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct UP900CL12B_DEVDATA 
{
    UP900CL12B_CAMERA * pCamera;
    E_UP900CL12B_FUNC   function;
    dbCommon * pRecord;
    void * pArg;
} UP900CL12B_DEVDATA;

/* This function will be called by all device support */
/* The memory for UP900CL12B_DEVDATA will be malloced inside */
static int UP900CL12B_DevData_Init(dbCommon * precord, E_EPICS_RTYPE rtype, char * ioString)
{
    UP900CL12B_DEVDATA *   pdevdata;

    UP900CL12B_CAMERA * pCamera;

    char    devname[40];
    char    param[40];
    E_UP900CL12B_FUNC    funcflag = 0;

    int     count;
    int     loop;

    /* param check */
    if(precord == NULL || ioString == NULL)
    {
        if(!precord) errlogPrintf("No legal record pointer!\n");
        if(!ioString) errlogPrintf("No INP/OUT field for record %s!\n", precord->name);
        return -1;
    }

    /* analyze INP/OUT string */
    count = sscanf(ioString, "%[^:]:%[^:]", devname, param);
    if (count != 2)
    {
        errlogPrintf("Record %s INP/OUT string %s format is illegal!\n", precord->name, ioString);
        return -1;
    }

    pCamera = (UP900CL12B_CAMERA *)devCameraFindByName(devname, CAMERA_MODEL_NAME);
    if(pCamera == NULL)
    {
      errlogPrintf("Can't find %s camera [%s] for record [%s] (param=%s)!\n", CAMERA_MODEL_NAME, devname, precord->name, param);
        return -1;
    }

    for(loop=0; loop<N_PARAM_MAP; loop++)
    {
        if( 0 == strcmp(param_map[loop].param, param) && param_map[loop].rtype == rtype)
        {
            funcflag = param_map[loop].funcflag;
            break;
        }
    }
    if(loop >= N_PARAM_MAP)
    {
        errlogPrintf("Record %s param %s is illegal!\n", precord->name, param);
        return -1;
    }

    pdevdata = (UP900CL12B_DEVDATA *)callocMustSucceed(1, sizeof(UP900CL12B_DEVDATA), "allocate memory for UP900CL12B_DEVDATA");

    pdevdata->pCamera = pCamera;
    pdevdata->function = funcflag;
    pdevdata->pRecord = precord;
    pdevdata->pArg = NULL;

    precord->dpvt = (void *)pdevdata;
    return 0;
}

/********* ai record *****************/
static long init_ai( struct aiRecord * pai)
{
    pai->dpvt = NULL;

    if (pai->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pai, "devAiEDTCL_UP900_12B Init_record, Illegal INP");
        pai->pact=TRUE;
        return (S_db_badField);
    }

    if( UP900CL12B_DevData_Init((dbCommon *)pai, EPICS_RTYPE_AI, pai->inp.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pai->name);
        recGblRecordError(S_db_badField, (void *) pai, "Init devdata Error");
        pai->pact = TRUE;
        return (S_db_badField);
    }

    return 0;
}

static long read_ai(struct aiRecord * pai)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    if(!(pai->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(pai->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_AI_CurFwhmX:
    case UP900CL12B_AI_CurFwhmY:
    case UP900CL12B_AI_CurCtrdX:
    case UP900CL12B_AI_CurCtrdY:
    case UP900CL12B_AI_HisFwhmX:
    case UP900CL12B_AI_HisFwhmY:
    case UP900CL12B_AI_HisCtrdX:
    case UP900CL12B_AI_HisCtrdY:
        /* Calculate projiection, FWHM, centroid ... */
      return -1;
    case UP900CL12B_AI_SaveMax:
      pai->val = pCamera->saveTimeMax;
      break;
    case UP900CL12B_AI_SaveMin:
      pai->val = pCamera->saveTimeMin;
      break;

    default:
        return -1;
    }

    return 2;	/* no conversion */
}

/********* bo record *****************/
static long init_bo( struct boRecord * pbo)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    pbo->dpvt = NULL;

    if (pbo->out.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoEDTCL_UP900_12B Init_record, Illegal OUT");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    pbo->mask = 0;

    if( UP900CL12B_DevData_Init((dbCommon *)pbo, EPICS_RTYPE_BO, pbo->out.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pbo->name);
        recGblRecordError(S_db_badField, (void *) pbo, "Init devdata Error");
        pbo->pact = TRUE;
        return (S_db_badField);
    }

    pdevdata = (UP900CL12B_DEVDATA *)(pbo->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_BO_SaveImage:
        pbo->rval = pCamera->saveImage;
        pbo->udf = FALSE;
        pbo->stat = pbo->sevr = NO_ALARM;
        break;
    case UP900CL12B_BO_DAQReadReset:
      pbo->rval = 0;
        pbo->udf = FALSE;
        pbo->stat = pbo->sevr = NO_ALARM;
        break;
    case UP900CL12B_BO_TriggerDAQ:
      pCamera->triggerDAQ = 0;
      pbo->rval = pCamera->triggerDAQ;
        pbo->udf = FALSE;
        pbo->stat = pbo->sevr = NO_ALARM;
        break;
    case UP900CL12B_BO_TriggerTS:
      gettimeofday(&(pCamera->triggerTS), 0);
      pbo->rval = 0;
      pbo->udf = FALSE;
      pbo->stat = pbo->sevr = NO_ALARM;
      break;
    default:
        break;
    }
    return 0;
}

static long write_bo(struct boRecord * pbo)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    if(!(pbo->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(pbo->dpvt);
    pCamera = pdevdata->pCamera;

    IMAGE_BUF *pImageBuf;
    pImageBuf = pCamera->historyBuf + pCamera->historyBufIndex;

    switch(pdevdata->function)
    {
    case UP900CL12B_BO_SaveImage:
        pCamera->saveImage = pbo->rval;
        break;
    case UP900CL12B_BO_DAQReadReset:
        pCamera->maxImageDelayUs = 0;
        pCamera->minImageDelayUs = 0xFFFFFFF;
        pCamera->avgImageDelayUs = 0;
        break;
    case UP900CL12B_BO_TriggerTS:
      gettimeofday(&(pCamera->triggerTS), 0);
      if (pCamera->statusDAQ == DAQ_ACQUIRING_IMAGES) {
	epicsTimeGetEvent(&(pImageBuf->triggerTimeStamp), pCamera->imageTimestampEvent);
/* 	printf("buf (%d)\n",  pCamera->historyBufIndex); */
	pCamera->numTriggersDAQ++;
      }
      else {
	pCamera->numTriggersDAQ = 0;
      }
      break;
    case UP900CL12B_BO_TriggerDAQ:
      /** If TRIGGER_DAQ is enabled */
      if (pbo->rval == 1) {
	/**
	 * Check if DAQ has finished -> triggerDAQ is set to 0 when done
	 * Signal in the STATUS_DAQ PV that system is ready for another DAQ
	 */
	if (pCamera->triggerDAQ == 0) {
	  epicsMutexLock(pCamera->historyBufMutexLock);
	  pCamera->historyBufIndex = 0;
	  pCamera->historyBufFull = 0; /* "Clear" buffer for next DAQ cycle */
	  pCamera->statusDAQ = DAQ_ACQUIRING_IMAGES;
	  pCamera->numImagesDAQCnt = 0;
	  /*pCamera->saveImage = 1; *//* enable saving to the image buffer instead of the ping-pong */
	  scanIoRequest(pCamera->ioscanpvt);
/* 	  printf("Got trigger - DAQ Starts now. STATUS=%d\n",pCamera->statusDAQ); */
	  epicsMutexUnlock(pCamera->historyBufMutexLock);
	}
	/** If not done yet, keep the trigger enabled */
	pCamera->triggerDAQ = 1;
      }
      /**
       * If TRIGGER_DAQ is disabled (0), then check if last DAQ finished,
       * and set triggerDAQ to zero again (this is not really needed)
       */
      else {
	if (pCamera->statusDAQ == DAQ_READY) {
	  pCamera->triggerDAQ = 0;
	  pCamera->saveImage = 0;
	  pCamera->numTriggersDAQ = 0;
	}
      }

      break;
    default:
        return -1;
    }

    return 0;
}

/********* li record *****************/
static long init_li( struct longinRecord * pli)
{
    pli->dpvt = NULL;

    if (pli->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiEDTCL_UP900_12B Init_record, Illegal INP");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    if( UP900CL12B_DevData_Init((dbCommon *)pli, EPICS_RTYPE_LI, pli->inp.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pli->name);
        recGblRecordError(S_db_badField, (void *) pli, "Init devdata Error");
        pli->pact = TRUE;
        return (S_db_badField);
    }

    return 0;
}

static long get_li_ioinfo(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt) {
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    if(!(precord->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(precord->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_LI_StatusDAQ:
      *ppvt = pCamera->ioscanpvt;
      break;
    default:
        return -1;
    }

    return 0;
}


static long read_li(struct longinRecord * pli)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    if(!(pli->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(pli->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_LI_NumOfCol:
        pli->val = pCamera->numOfCol;
        break;
    case UP900CL12B_LI_NumOfRow:
        pli->val = pCamera->numOfRow;
        break;
    case UP900CL12B_LI_NumOfBits:
        pli->val = pCamera->numOfBits;
        break;
    case UP900CL12B_LI_FrameRate:
        pli->val = pCamera->frameCounts;
        pCamera->frameCounts = 0;
        break;
    case UP900CL12B_LI_StartPixelX:
        pli->val = pCamera->startPixelX;
        break;
    case UP900CL12B_LI_StartPixelY:
        pli->val = pCamera->startPixelY;
        break;
    case UP900CL12B_LI_NumPixelsX:
        pli->val = pCamera->nPixelsX;
        break;
    case UP900CL12B_LI_NumPixelsY:
        pli->val = pCamera->nPixelsY;
        break;
    case UP900CL12B_LI_StatusDAQ:
      pli->val = pCamera->statusDAQ;
      break;
    case UP900CL12B_LI_DAQDupId:
      pli->val = pCamera->maxImageDelayUs;
      break;
    case UP900CL12B_LI_DAQReadMax:
      pli->val = pCamera->maxImageDelayUs;
      break;
    case UP900CL12B_LI_DAQReadMin:
      pli->val = pCamera->minImageDelayUs;
      break;
    case UP900CL12B_LI_DAQReadAvg:
      pli->val = pCamera->avgImageDelayUs;
      break;
    case UP900CL12B_LI_DAQImgCnt:
      pli->val = pCamera->numImagesDAQCnt;
      break;
    case UP900CL12B_LI_ReadMax:
      pli->val = pCamera->readTimeMax;
      break;
    case UP900CL12B_LI_ReadMin:
      pli->val = pCamera->readTimeMin;
      break;
    default:
        return -1;
    }

    return 0;
}

/********* lo record *****************/
static long init_lo( struct longoutRecord * plo)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    plo->dpvt = NULL;

    if (plo->out.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)plo, "devLoEDTCL_UP900_12B Init_record, Illegal OUT");
        plo->pact=TRUE;
        return (S_db_badField);
    }

    if( UP900CL12B_DevData_Init((dbCommon *)plo, EPICS_RTYPE_LO, plo->out.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", plo->name);
        recGblRecordError(S_db_badField, (void *) plo, "Init devdata Error");
        plo->pact = TRUE;
        return (S_db_badField);
    }

    pdevdata = (UP900CL12B_DEVDATA *)(plo->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_LO_HisIndex:
        plo->val = pCamera->historyBufReadOffset;
        plo->hopr = plo->drvh = 0;
        plo->lopr = plo->drvl = 2 - NUM_OF_FRAMES; /* one frame is always reserved */
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    case UP900CL12B_LO_StartPixelX:
        epicsMutexLock(pCamera->mutexLock);
        plo->val = pCamera->startPixelX;
        epicsMutexUnlock(pCamera->mutexLock);
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    case UP900CL12B_LO_StartPixelY:
        epicsMutexLock(pCamera->mutexLock);
        plo->val = pCamera->startPixelY;
        epicsMutexUnlock(pCamera->mutexLock);
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    case UP900CL12B_LO_NumPixelsX:
        epicsMutexLock(pCamera->mutexLock);
        plo->val = pCamera->nPixelsX;
        epicsMutexUnlock(pCamera->mutexLock);
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    case UP900CL12B_LO_NumPixelsY:
        epicsMutexLock(pCamera->mutexLock);
        plo->val = pCamera->nPixelsY;
        epicsMutexUnlock(pCamera->mutexLock);
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    case UP900CL12B_LO_NumImagesDAQ:
        epicsMutexLock(pCamera->mutexLock);
        plo->val = pCamera->numImagesDAQ;
        epicsMutexUnlock(pCamera->mutexLock);
        plo->udf = FALSE;
        plo->stat = plo->sevr = NO_ALARM;
        break;
    default:
        break;
    }
    return 0;
}

static long write_lo(struct longoutRecord * plo)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    if(!(plo->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(plo->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_LO_HisIndex:
        pCamera->historyBufReadOffset = plo->val;/* we just take the vaule, the record will limit the range and other read will mark invalid if it is out of range */
        break;
    case UP900CL12B_LO_StartPixelX:
        if(plo->val < 0)
            pCamera->startPixelX = 0;
        else if(plo->val > (pCamera->numOfCol - pCamera->nPixelsX))
            pCamera->startPixelX = pCamera->numOfCol - pCamera->nPixelsX;
        else
            pCamera->startPixelX = plo->val;
        
        plo->val = pCamera->startPixelX;
        break;
    case UP900CL12B_LO_StartPixelY:
        if(plo->val < 0)
            pCamera->startPixelY = 0;
        else if(plo->val >= (pCamera->numOfRow - pCamera->nPixelsY))
            pCamera->startPixelY = pCamera->numOfRow - pCamera->nPixelsY;
        else
            pCamera->startPixelY = plo->val;
        
        plo->val = pCamera->startPixelY;
        break;
    case UP900CL12B_LO_NumPixelsX:
        if(plo->val < 1)
            pCamera->nPixelsX = 1;
        else if(plo->val > (pCamera->numOfCol - pCamera->startPixelX))
            pCamera->nPixelsX = pCamera->numOfCol - pCamera->startPixelX;
        else
            pCamera->nPixelsX = plo->val;
        
        plo->val = pCamera->nPixelsX;
        break;
    case UP900CL12B_LO_NumPixelsY:
        if(plo->val < 1)
            pCamera->nPixelsY = 1;
        else if(plo->val > (pCamera->numOfRow - pCamera->startPixelY))
            pCamera->nPixelsY = pCamera->numOfRow - pCamera->startPixelY;
        else
            pCamera->nPixelsY = plo->val;
        
        plo->val = pCamera->nPixelsY;
        break;
    case UP900CL12B_LO_NumImagesDAQ:
        if(plo->val < 1)
	  pCamera->numImagesDAQ = 1;
        else if(plo->val > NUM_OF_FRAMES)
	  pCamera->numImagesDAQ = 100;
        else
            pCamera->numImagesDAQ = plo->val;
        
        plo->val = pCamera->numImagesDAQ;
	printf("Number of images %d\n", plo->val);
        break;
    case UP900CL12B_LO_ImageTimestampEvent:
      epicsMutexLock(pCamera->historyBufMutexLock);
      pCamera->imageTimestampEvent = plo->val;
      epicsMutexUnlock(pCamera->historyBufMutexLock);
      plo->udf = FALSE;
      plo->stat = plo->sevr = NO_ALARM;
      break;
    default:
        return -1;
    }

    return 0;
}

/********* waveform record *****************/
static long init_wf(struct waveformRecord *pwf)
{
    pwf->dpvt = NULL;

    if (pwf->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B Init_record, Illegal INP");
        pwf->pact=TRUE;
        return (S_db_badField);
    }

    if( UP900CL12B_DevData_Init((dbCommon *)pwf, EPICS_RTYPE_WF, pwf->inp.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pwf->name);
        recGblRecordError(S_db_badField, (void *) pwf, "Init devdata Error");
        pwf->pact = TRUE;
        return (S_db_badField);
    }

    return 0;
}

static long read_wf(struct waveformRecord * pwf)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    IMAGE_BUF * pCurImageBuf;	/* current image buffer, could be ping-pong or circular buffer */
    IMAGE_BUF * pHisImageBuf;	/* history image buffer, must be from circular buffer */

    int numOfAvailFrames = 0;
    int wflen = 0;

    if(!(pwf->dpvt)) return -1;

    pdevdata = (UP900CL12B_DEVDATA *)(pwf->dpvt);
    pCamera = pdevdata->pCamera;

    if(pCamera->saveImage)
    {/* current image is from circular buffer */
        epicsMutexLock(pCamera->historyBufMutexLock);
        if(!pCamera->historyBufFull && pCamera->historyBufIndex == 0)
        {
            pCurImageBuf = NULL;
        }
        else
        {
            pCurImageBuf = pCamera->historyBuf + ((pCamera->historyBufIndex + NUM_OF_FRAMES - 1) % NUM_OF_FRAMES);
        }
        epicsMutexUnlock(pCamera->historyBufMutexLock);
    }
    else
    {/* current image is from ping-pong buffer */
        pCurImageBuf = pCamera->pingpongBuf + (1 - pCamera->pingpongFlag);
    }

    epicsMutexLock(pCamera->historyBufMutexLock);
    if(pCamera->historyBufFull) numOfAvailFrames = NUM_OF_FRAMES - 1;/* reserve last frame always */
    else numOfAvailFrames = pCamera->historyBufIndex;

    if( (numOfAvailFrames + pCamera->historyBufReadOffset) <= 0 )
    {
        pHisImageBuf = NULL;
    }
    else
    {
        pHisImageBuf = pCamera->historyBuf + ((pCamera->historyBufIndex + NUM_OF_FRAMES - 1 + pCamera->historyBufReadOffset) % NUM_OF_FRAMES);
    }
    epicsMutexUnlock(pCamera->historyBufMutexLock);

    switch(pdevdata->function)
    {
    case UP900CL12B_WF_CurImage:
        if(pwf->ftvl != DBF_USHORT || pwf->nelm != pCamera->imageSize/sizeof(unsigned short int))
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pCurImageBuf)
        {
#if 0
            memcpy((void*)(pwf->bptr), (void*)pCurImageBuf->pImage, pCamera->imageSize);
            wflen = pCamera->imageSize/sizeof(unsigned short int);
#else
            /* Do line copy */
            int loopline;
            char * pdest = (char *)(pwf->bptr);
            epicsMutexLock(pCamera->mutexLock);
            for(loopline = 0; loopline < pCamera->nPixelsY; loopline++)
            {
                memcpy((void*)pdest, (void*)(pCurImageBuf->ppRow[loopline+pCamera->startPixelY]+pCamera->startPixelX), pCamera->nPixelsX*sizeof(unsigned short int));
                pdest += pCamera->nPixelsX * sizeof(unsigned short int);
            }
            wflen = pCamera->nPixelsX * pCamera->nPixelsY;
            epicsMutexUnlock(pCamera->mutexLock);
#endif
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pCurImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    case UP900CL12B_WF_HisImage:
        if(pwf->ftvl != DBF_USHORT || pwf->nelm != pCamera->imageSize/sizeof(unsigned short int))
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pHisImageBuf)
        {
#if 0
            memcpy((void*)(pwf->bptr), (void*)pHisImageBuf->pImage, pCamera->imageSize);
            wflen = pCamera->imageSize/sizeof(unsigned short int);
#else
            /* Do line copy */
            int loopline;
            char * pdest = (char *)(pwf->bptr);
            epicsMutexLock(pCamera->mutexLock);
            for(loopline = 0; loopline < pCamera->nPixelsY; loopline++)
            {
                memcpy((void*)pdest, (void*)(pHisImageBuf->ppRow[loopline+pCamera->startPixelY]+pCamera->startPixelX), pCamera->nPixelsX*sizeof(unsigned short int));
                pdest += pCamera->nPixelsX * sizeof(unsigned short int);
            }
            wflen = pCamera->nPixelsX * pCamera->nPixelsY;
            epicsMutexUnlock(pCamera->mutexLock);
#endif
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pHisImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    case UP900CL12B_WF_CurProjX:
        if(pwf->ftvl != DBF_LONG || pwf->nelm != pCamera->numOfCol)
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pCurImageBuf)
        {
            memcpy((void*)(pwf->bptr), (void*)pCurImageBuf->pProjectionX, pCamera->numOfCol*sizeof(unsigned int));
            wflen = pCamera->numOfCol;
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pCurImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    case UP900CL12B_WF_CurProjY:
        if(pwf->ftvl != DBF_LONG || pwf->nelm != pCamera->numOfRow)
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pCurImageBuf)
        {
            memcpy((void*)(pwf->bptr), (void*)pCurImageBuf->pProjectionY, pCamera->numOfRow*sizeof(unsigned int));
            wflen = pCamera->numOfRow;
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pCurImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    case UP900CL12B_WF_HisProjX:
        if(pwf->ftvl != DBF_LONG || pwf->nelm != pCamera->numOfCol)
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pHisImageBuf)
        {
            memcpy((void*)(pwf->bptr), (void*)pHisImageBuf->pProjectionX, pCamera->numOfCol*sizeof(unsigned int));
            wflen = pCamera->numOfCol;
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pHisImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    case UP900CL12B_WF_HisProjY:
        if(pwf->ftvl != DBF_LONG || pwf->nelm != pCamera->numOfRow)
        {
            recGblRecordError(S_db_badField, (void *)pwf, "devWfEDTCL_UP900_12B read_record, Illegal FTVL or NELM field");
            pwf->pact=TRUE;
            return (S_db_badField);
        }
        if(pHisImageBuf)
        {
            memcpy((void*)(pwf->bptr), (void*)pHisImageBuf->pProjectionY, pCamera->numOfRow*sizeof(unsigned int));
            wflen = pCamera->numOfRow;
            if(pwf->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pwf->time = pHisImageBuf->timeStamp;
        }
        else
        {
            wflen = 0;
            recGblSetSevr(pwf,READ_ALARM,INVALID_ALARM);
        }
        break;
    default:
        wflen = 0;
        break;
    }

    if(pwf->rarm)	pwf->rarm=0;	/* reset RARM */

    pwf->nord=wflen;
    pwf->udf=FALSE;
    return 0;
}

/********* stringout record *****************/
static long init_so(struct stringoutRecord *pso)
{
    UP900CL12B_DEVDATA * pdevdata;
    UP900CL12B_CAMERA * pCamera;

    pso->dpvt = NULL;

    if (pso->out.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pso, "devBoEDTCL_UP900_12B Init_record, Illegal OUT");
        pso->pact=TRUE;
        return (S_db_badField);
    }

    if( UP900CL12B_DevData_Init((dbCommon *)pso, EPICS_RTYPE_SO, pso->out.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pso->name);
        recGblRecordError(S_db_badField, (void *) pso, "Init devdata Error");
        pso->pact = TRUE;
        return (S_db_badField);
    }

    pdevdata = (UP900CL12B_DEVDATA *)(pso->dpvt);
    pCamera = pdevdata->pCamera;

    switch(pdevdata->function)
    {
    case UP900CL12B_SO_SaveImageDir:
      strcpy(pso->val, "unknown");
      pso->udf = FALSE;
      pso->stat = pso->sevr = NO_ALARM;
      break;
    case UP900CL12B_SO_ImageName:
      strcpy(pso->val, pCamera->pCameraName);
      pso->udf = FALSE;
      pso->stat = pso->sevr = NO_ALARM;
      break;
    default:
        break;
    }
    return 0;
}

static long write_so(struct stringoutRecord *pso)
{
  UP900CL12B_DEVDATA * pdevdata;
  UP900CL12B_CAMERA * pCamera;

  if(!(pso->dpvt)) {
    return -1;
  }
  
  pdevdata = (UP900CL12B_DEVDATA *)(pso->dpvt);
  pCamera = pdevdata->pCamera;
  
  struct stat stat_buf;

  switch(pdevdata->function)
    {
    case UP900CL12B_SO_SaveImageDir:
      strncpy(pCamera->saveImageDir, pso->val, 40);
      if(strncmp("FTP1:", pCamera->saveImageDir, 5) == 0) {
	sprintf(pCamera->saveImageDir, "/FTP/anonymous:pass/@172.27.72.30%s",&(pso->val[5]));
      }
      if(strncmp("FTP2:", pCamera->saveImageDir, 5) == 0) {
	sprintf(pCamera->saveImageDir, "/FTP/anonymous:pass/@172.27.72.31%s",&(pso->val[5]));
      }
      if(strncmp("FTP3:", pCamera->saveImageDir, 5) == 0) {
	sprintf(pCamera->saveImageDir, "/FTP/anonymous:pass/@172.27.72.32%s",&(pso->val[5]));
      }
      break;
    case UP900CL12B_SO_ImageName:
      strncpy(pCamera->imageName, pso->val, 40);
      break;
    default:
      return -1;
    }
  
  return 0;
}

/*============================================*/
static int image12b_noise_reduce(unsigned char * image, int image_size, float threshold_ratio)
{
    int loop;
    int max_pixel=0;
    int min_pixel=4096;
    int threshold=0;

    unsigned short int * pimage;

    if(!image) return -1;

    pimage = (unsigned short int *)image;
    for(loop=0; loop<image_size/sizeof(unsigned short int); loop++)
    {
        max_pixel = max(max_pixel, pimage[loop]);
        /*min_pixel = min(min_pixel, pimage[loop]);*/
    }
    threshold = max_pixel * threshold_ratio;
    for(loop=0; loop<image_size/sizeof(unsigned short int); loop++) pimage[loop] = (pimage[loop] <= threshold) ? 0 : (pimage[loop]-threshold);
    return 0;
}

static int image12b_projection_calc(const unsigned char * image, int * proj_H, int num_col, int * proj_V, int num_row)
{
    int loop, subloop;
    const unsigned short int  ** Y;

    unsigned short int * pimage;

    if(!image) return -1;
    if(!proj_H) return -1;
    if(!proj_V) return -1;

    pimage = (unsigned short int *)image;

    Y = malloc(num_row*sizeof(unsigned short int *));
    if(!Y) return -1;

    Y[0] = pimage;
    for(loop=1; loop<num_row; loop++) Y[loop] = Y[loop-1] + num_col;

    for(loop=0; loop<num_row; loop++)
    {
        proj_V[loop] = 0;
        for(subloop=0; subloop < num_col; subloop++)
        {
            proj_V[loop] += Y[loop][subloop];
        }
    }

    for(loop=0; loop<num_col; loop++)
    {
        proj_H[loop] = 0;
        for(subloop=0; subloop < num_row; subloop++)
        {
            proj_H[loop] += Y[subloop][loop];
        }
    }

    free(Y);
    return 0;
}

static int image12b_centroid_calc(int * proj_H, int num_col, int * proj_V, int num_row, double * cen_H, double * cen_V)
{
    int loop;
    double sum_image=0.0;	/* About 1.3M pixels, so 21bits. The 12bits per pixel, to up to 33 bits */
    double sum=0.0;

    if(!proj_H) return -1;
    if(!proj_V) return -1;
    if(!cen_H) return -1;
    if(!cen_V) return -1;

    /* cakculate centroid H */
    sum_image = 0.0;
    sum = 0.0;
    for(loop=0; loop<num_col; loop++)
    {
        sum_image += proj_H[loop];
        sum += loop * proj_H[loop];
    }
    *cen_H = sum/sum_image;

    /* cakculate centroid V */
    sum = 0.0;
    for(loop=0; loop<num_row; loop++)
    {
        sum += loop * proj_V[loop];
    }
    *cen_V = sum/sum_image;

    return 0;
}

