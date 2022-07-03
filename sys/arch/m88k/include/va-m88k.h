/*	$OpenBSD: va-m88k.h,v 1.9 2006/04/09 03:07:53 deraadt Exp $	*/

#if defined (_STDARG_H) || defined (_VARARGS_H)

#ifdef _STDARG_H /* stdarg.h support */
#define	va_start(ap, last) \
__extension__ ({ \
  (ap) = (va_list)__builtin_alloca(sizeof(__builtin_va_list)); \
  __builtin_stdarg_start(*(ap), (last)); \
  })
#else /* varargs.h support */
#define	__va_ellipsis	...
#define	va_start(ap) \
__extension__ ({ \
  (ap) = (va_list)__builtin_alloca(sizeof(__builtin_va_list)); \
  __builtin_varargs_start(*(ap)); \
  })
#define	__va_alist_t	__builtin_va_alist_t
#define	va_alist	__builtin_va_alist
#define	va_dcl		__va_alist_t __builtin_va_alist; __va_ellipsis
#endif /* _STDARG_H */

#define	va_arg(ap, type)	__builtin_va_arg(*(ap), type)
#define	va_end(ap)		__builtin_va_end(*(ap))

#define	__va_copy(dest, src)	__builtin_va_copy(*(dest), *(src))

#if !defined(_ANSI_SOURCE) &&						\
    (defined(_ISOC99_SOURCE) || (__STDC_VERSION__ - 0) >= 199901L ||	\
     defined(_NETBSD_SOURCE))
#define	va_copy(dest, src)	__va_copy((dest), (src))
#endif

#endif /* defined (_STDARG_H) || defined (_VARARGS_H) */
