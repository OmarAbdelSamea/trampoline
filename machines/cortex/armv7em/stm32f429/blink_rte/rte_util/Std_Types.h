#ifndef _STD_TYPE_H
#define _STD_TYPE_H


#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char  boolean;

//#ifndef __A_STD_TYPE_H

//	typedef unsigned char  bit8;
//	typedef unsigned short bit16;
//	typedef unsigned long  bit32;
//
//	typedef unsigned int   uint;
//	typedef signed int     sint;
//	typedef double         real;

	typedef unsigned char  uint8;
	typedef signed char    sint8;
	typedef unsigned short uint16;
	typedef signed short   sint16;
	typedef unsigned long  uint32;
	typedef signed long    sint32;
	typedef float          real32;
	typedef double         real64;

	typedef float          float32;

	#if !defined(_C166)
		/* tasking compiler uses bit type */
		typedef unsigned char  bit;
	#endif

	#if defined(__DCC__)  || defined(__use64integers__) && !defined(__QA_C_CHECKER)
		/* diab compiler supports long long type. */
	/* explicitly define  __use64integers__ for other compilers supporting this type. */
		typedef unsigned long long uint64;
		typedef signed long long   sint64;
	#endif

		typedef uint8 Std_ReturnType;
//#endif /* __A_STD_TYPE_H */

//#ifndef TRUE                                             /* conditional check */
//  #define TRUE (!FALSE)     /* This is a deviation to AUTOSAR, but we need it */
//                            /* because ESP8_LEGACY defines it like that and   */
//                            /* we have modules including both header files.   */
//#endif
//
//#ifndef FALSE                                            /* conditional check */
// #define FALSE     0
//#endif

#ifndef false
    #define false (0)
#endif
#ifndef true
    #define true  (!false)
#endif
#ifndef FALSE
	#define FALSE   (0)
#endif
#ifndef TRUE
	#define TRUE    (!FALSE)
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif
