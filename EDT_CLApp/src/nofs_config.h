#ifndef _nofs_config_h_
#define _nofs_config_h_

char *ptm6710cl_cfg[] =
#include "ptm6710cl.cfg.h"

char *ptm6710cl_pw_cfg[] =
#include "ptm6710cl_pw.cfg.h"

char *up900cl12b_cfg[] =
#include "up900cl12b.cfg.h"

char *ptm4200cl_pw_cfg[] =
#include "ptm4200cl_pw.cfg.h"

char *up685cl8b_cfg[] =
#include "up685cl8b.cfg.h"

char *up685cl8b_pw_cfg[] =
#include "up685cl8b_pw.cfg.h"

char *up685cl10b_cfg[] =
#include "up685cl10b.cfg.h"

char *up685cl10b_pw_cfg[] =
#include "up685cl10b_pw.cfg.h"

#define MAPCONFIG(cf, vx_p) \
     if (strcmp(cf, "ptm6710cl") == 0) \
        vx_p = ptm6710cl_cfg ; \
     else if (strcmp(cf, "ptm6710cl_pw") == 0) \
        vx_p = ptm6710cl_pw_cfg ; \
     else if (strcmp(cf, "up900cl12b") == 0) \
        vx_p = up900cl12b_cfg ; \
     else if (strcmp(cf, "ptm4200cl_pw") == 0) \
        vx_p = ptm4200cl_pw_cfg ; \
     else if (strcmp(cf, "up685cl8b") == 0) \
        vx_p = up685cl8b_cfg ; \
     else if (strcmp(cf, "up685cl8b_pw") == 0) \
        vx_p = up685cl8b_pw_cfg ; \
     else if (strcmp(cf, "up685cl10b") == 0) \
        vx_p = up685cl10b_cfg ; \
     else if (strcmp(cf, "up685cl10b_pw") == 0) \
        vx_p = up685cl10b_pw_cfg ; \
     else vx_p = NULL ;

#endif /* _nofs_config_h_ */

