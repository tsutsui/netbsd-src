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

/* Relocation numbers and processor tags used by the ELF runtime linker. */
#define R_88K_NONE              0
#define R_88K_COPY              1
#define R_88K_GOTP_ENT          2
#define R_88K_BBASED_32        16
#define R_88K_32               32

#define DT_88K_ADDRBASE        0x70000001
#define DT_88K_PLTSTART        0x70000002
#define DT_88K_PLTEND          0x70000003
#define DT_88K_TDESC           0x70000004

#define R_TYPE(name)           __CONCAT(R_88K_,name)

#endif /* ! _M88K_ELF_MACHDEP_H_ */
