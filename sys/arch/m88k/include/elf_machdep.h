/* $NetBSD$ */

#ifndef _M88K_ELF_MACHDEP_H_
#define _M88K_ELF_MACHDEP_H_

#define ELF32_MACHDEP_ENDIANNESS        ELFDATA2MSB
#define ELF32_MACHDEP_ID_CASES                                          \
                case EM_88K:                                           \
                        break;

#define ELF64_MACHDEP_ENDIANNESS        XXX     /* break compilation */
#define ELF64_MACHDEP_ID_CASES                                          \
                /* no 64-bit ELF machine types supported */

#define ELF32_MACHDEP_ID        EM_88K

#define ARCH_ELFSIZE		32

/* XXX - missing relocs - TKM */

#endif /* ! _M88K_ELF_MACHDEP_H_ */
