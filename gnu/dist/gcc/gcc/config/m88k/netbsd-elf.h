/* Definitions of target machine for GNU compiler,
for m88k NetBSD platforms using the ELF object format.
Copyright (C) 2006 Free Software Foundation, Inc.

This file is derived from <m68k/netbsd-elf.h> and <m88k/openbsd.h>

This file is part of GNU CC.

GNU CC is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

GNU CC is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with GNU CC; see the file COPYING.  If not, write to
the Free Software Foundation, 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.  */

/* Identify the compiler.  */
#undef  VERSION_INFO1
#define VERSION_INFO1 "88open ABI"

/* Default switches */
#undef	TARGET_DEFAULT
#define TARGET_DEFAULT	(MASK_CHECK_ZERO_DIV | \
			 MASK_NO_POUND_SIGN  | \
			 MASK_SVR4)

/* Make sure this is undefined.  */
#undef CPP_PREDEFINES

#define TARGET_OS_CPP_BUILTINS()		\
do						\
{						\
    NETBSD_OS_CPP_BUILTINS_ELF();		\
    builtin_define ("__CLASSIFY_TYPE__=2");	\
    builtin_define ("__m88000__");		\
    builtin_define ("__m88k__");		\
    builtin_define ("__SVR4_ABI__");		\
    builtin_define ("__motorola__");		\
    builtin_assert ("cpu=m88k");		\
    builtin_assert ("machine=m88k");		\
}						\
while (0)

#define EXTRA_SPECS \
  { "cpp_cpu_spec",         CPP_CPU_SPEC }, \
  { "netbsd_cpp_spec",      NETBSD_CPP_SPEC }, \
  { "netbsd_entry_point",   NETBSD_ENTRY_POINT },

/* If -m88000 is in effect, add -Dmc88000; similarly for -m88100 and -m88110.
However, reproduce the effect of -Dmc88100 previously in CPP_PREDEFINES.
Here, the CPU_DEFAULT is assumed to be -m88100.  */
#define CPP_CPU_SPEC \
  "%{m88000:-D__mc88000__} \
   %{!m88000:%{m88100:%{m88110:-D__mc88000__}}} \
   %{!m88000:%{!m88100:%{m88110:-D__mc88110__}}} \
   %{!m88000:%{!m88110:-D__mc88100__ -D__mc88100}}"

#undef CPP_SPEC
#define CPP_SPEC \
  "%(netbsd_cpp_spec) %(cpp_cpu_spec)"

#undef LINK_SPEC
#define LINK_SPEC NETBSD_LINK_SPEC_ELF

#define NETBSD_ENTRY_POINT "_start"

/* Layout of source language data types. */

/* This must agree with <machine/ansi.h> */
#undef SIZE_TYPE
#define SIZE_TYPE "unsigned int"

#undef PTRDIFF_TYPE
#define PTRDIFF_TYPE "int"

#undef WCHAR_TYPE
#define WCHAR_TYPE "int"

#undef WCHAR_TYPE_SIZE
#define WCHAR_TYPE_SIZE 32

/* Override svr4.h and m88k.h.  */
#undef  INIT_SECTION_ASM_OP
#define INIT_SECTION_ASM_OP "\tsection\t.init,\"xa\""
#undef  FINI_SECTION_ASM_OP
#define FINI_SECTION_ASM_OP "\tsection\t.fini,\"xa\""

/* Define the pseudo-ops used to switch to the .ctors and .dtors sections.
 
   Note that we want to give these sections the SHF_WRITE attribute
   because these sections will actually contain data (i.e. tables of
   addresses of functions in the current root executable or shared library
   file) and, in the case of a shared library, the relocatable addresses
   will have to be properly resolved/relocated (and then written into) by
   the dynamic linker when it actually attaches the given shared library
   to the executing process.  (Note that on SVR4, you may wish to use the
   `-z text' option to the ELF linker, when building a shared library, as
   an additional check that you are doing everything right.  But if you do
   use the `-z text' option when building a shared library, you will get
   errors unless the .ctors and .dtors sections are marked as writable
   via the SHF_WRITE attribute.)  */
 
#undef  CTORS_SECTION_ASM_OP
#define CTORS_SECTION_ASM_OP    "\tsection\t.ctors,\"aw\""
#undef  DTORS_SECTION_ASM_OP
#define DTORS_SECTION_ASM_OP    "\tsection\t.dtors,\"aw\""

