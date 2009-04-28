/************************************************************************
 *  Copyright (c) 1996 by Charles A. Measday                            *
 *                                                                      *
 *  Permission to use, copy, modify, and distribute this software       *
 *  and its documentation for any purpose and without fee is hereby     *
 *  granted, provided that the above copyright notice appear in all     *
 *  copies.  The author makes no representations about the suitability  *
 *  of this software for any purpose.  It is provided "as is" without   *
 *  express or implied warranty.                                        *
 ************************************************************************/
/*
@(#)  FILE: str_util.c  RELEASE: 1.13  DATE: 10/02/97, 14:25:41
*/
/*******************************************************************************

File:

    str_util.c


Author:    Alex Measday, ISI


Purpose:

    These are a collection of the string manipulation functions.
    Also, see the GET_UTIL functions.


Notes:

    These functions used to be part of the LIBALEX functions.  The following
    changes have been made:

      - Functions that take a length argument used to follow a convention
        that, if the length were zero, the function would determine the
        length by scanning the string for a null terminator.  This turned
        out to be a real nuisance if you had a need to handle zero-length
        strings ("").  The new convention is that,if the length argument
        is less than zero, the function will determine the length itself.

    These functions are reentrant under VxWorks (except for the global
    debug flag).


Procedures:

    strDestring() - resolves quote-delimited elements in a string.

*******************************************************************************/


#include  <ctype.h>			/* Standard character functions. */
#include  <errno.h>			/* System error definitions. */
#include  <stdio.h>			/* Standard I/O definitions. */
#include  <stdlib.h>			/* Standard C Library definitions. */
#include  <string.h>			/* Standard C string functions. */
#if !__STDC__ && defined(sun)
#    define  memmove(dest,src,length)  bcopy(src,dest,length)
#endif

#include  "get_util.h"			/* "Get Next" functions. */
#include  "vperror.h"			/* VPERROR() definitions. */
#include  "str_util.h"			/* String manipulation functions. */


int  str_util_debug = 0 ;		/* Global debug switch (1/0 = yes/no). */

char  *strDestring (

#    if __STDC__
        char  *string,
        int  length,
        const  char  *quotes)
#    else
        string, length, quotes)

        char  *string ;
        int  length ;
        char  *quotes ;
#    endif

{    /* Local variables. */
    char  *eos, rh_quote, *s ;




    if (string == NULL)  return ("") ;
    if (quotes == NULL)  quotes = "" ;

    if (length >= 0) {				/* Make copy of input string. */
        s = strndup (string, length) ;
        if (s == NULL) {
            vperror ("(strDestring) Error duplicating: \"%*s\"\nstrndup: ",
                     length, string) ;
            return (NULL) ;
        }
        string = s ;
    }


/* Scan the new argument and determine its length. */

    for (s = string ;  *s != '\0' ;  s++) {

        if (strchr (quotes, *s) == NULL)	/* Non-quote character? */
            continue ;

        switch (*s) {				/* Determine right-hand quote. */
        case '{':  rh_quote = '}' ;  break ;
        case '[':  rh_quote = ']' ;  break ;
        case '(':  rh_quote = ')' ;  break ;
        default:
            rh_quote = *s ;  break ;
            break ;
        }

        eos = strchr (s+1, rh_quote) ;		/* Locate right-hand quote. */
        if (eos == NULL)			/* Assume quote at null terminator. */
            eos = s + strlen (s) ;
        else					/* Pull down one character. */
            memmove (eos, eos+1, strlen (eos+1) + 1) ;

        memmove (s, s+1, strlen (s+1) + 1) ;	/* Pull down one character. */
        s = eos - 2 ;				/* 2 quotes gone! */

    }


/* Return the processed string to the calling routine. */

    return (string) ;

}

size_t  strTrim (

#    if __STDC__
        char  *string,
        int  length)
#    else
        string, length)

        char  *string ;
        int  length ;
#    endif

{    /* Local variables. */
    char  *s ;
    int  newLength ;



    newLength = (length < 0) ? strlen (string) : length ;
    s = string + newLength ;

    while ((s-- != string) &&
           ((*s == ' ') || (*s == '\t') || (*s == '\n')))
        newLength-- ;

    if (length < 0)  *++s = '\0' ;

    return (newLength) ;

}

