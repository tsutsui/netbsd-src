/* MC88k ELF support for BFD.
   Copyright 1998, 1999, 2000 Free Software Foundation, Inc.

   This file is part of BFD, the Binary File Descriptor library.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifndef _ELF_M88K_H
#define _ELF_M88K_H

#include "elf/reloc-macros.h"

/* Relocation types.  */

START_RELOC_NUMBERS (elf_m88k_reloc_type)
     RELOC_NUMBER (R_88K_NONE,       0)		/* No reloc */
     RELOC_NUMBER (R_88K_COPY,       1)
     RELOC_NUMBER (R_88K_GOTP_ENT,   2)
     RELOC_NUMBER (R_88K_8,          4)
     RELOC_NUMBER (R_88K_8S,         5)
     RELOC_NUMBER (R_88K_16S,        7)
     RELOC_NUMBER (R_88K_DISP16,     8)
     RELOC_NUMBER (R_88K_DISP26,     10)
     RELOC_NUMBER (R_88K_PLT_DISP26, 14)
     RELOC_NUMBER (R_88K_BBASED_32,  16)
     RELOC_NUMBER (R_88K_BBASED_32UA,17)
     RELOC_NUMBER (R_88K_BBASED_16H, 18)
     RELOC_NUMBER (R_88K_BBASED_16L, 19)
     RELOC_NUMBER (R_88K_ABDIFF_32,  24)
     RELOC_NUMBER (R_88K_ABDIFF_32UA,25)
     RELOC_NUMBER (R_88K_ABDIFF_16H, 26)
     RELOC_NUMBER (R_88K_ABDIFF_16L, 27)
     RELOC_NUMBER (R_88K_ABDIFF_16,  28)
     RELOC_NUMBER (R_88K_32,         32)
     RELOC_NUMBER (R_88K_32UA,       33)
     RELOC_NUMBER (R_88K_16H,        34)
     RELOC_NUMBER (R_88K_16L,        35)
     RELOC_NUMBER (R_88K_16,         36)
  /* These are GNU extensions to enable C++ vtable garbage collection.  */
  /* RELOC_NUMBER (R_88K_GNU_VTINHERIT, 23)
     RELOC_NUMBER (R_88K_GNU_VTENTRY, 24) */
END_RELOC_NUMBERS (R_88K_max)

#define EF_CPU32    0x00810000 XX!!
#define EF_M88000   0x01000000 XX!!

#endif
