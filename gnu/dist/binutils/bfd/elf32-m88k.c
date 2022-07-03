/* Motorola 88k-specific support for 32-bit ELF
   Copyright 1993, 1995, 1999 Free Software Foundation, Inc.

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
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#include "bfd.h"
#include "sysdep.h"
#include "libbfd.h"
#include "elf-bfd.h"
#include "elf/m88k.h"

static struct bfd_hash_entry *elf_m88k_link_hash_newfunc
  PARAMS ((struct bfd_hash_entry *, struct bfd_hash_table *, const char *));

static bfd_reloc_status_type elf_m88k_relocate_lo16
  PARAMS ((bfd *,  Elf_Internal_Rela *, bfd_byte *, bfd_vma));

static bfd_reloc_status_type elf_m88k_relocate_hi16
  PARAMS ((bfd *,  Elf_Internal_Rela *, bfd_byte *, bfd_vma));

/* This does not include any relocations, but should be good enough
   for GDB.  */

static reloc_howto_type howto_table[] = {
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_COPY,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_COPY",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_GOTP_ENT,   0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_GOTP_ENT",  FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_8,          0, 0, 8, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_8",         FALSE, 0, 0x000000ff,FALSE),
  HOWTO(R_88K_8S,         0, 0, 8, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_8S",        FALSE, 0, 0x000000ff,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_16S,        0, 1,16, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_16S",       FALSE, 0, 0x0000ffff,FALSE),
  HOWTO(R_88K_DISP16,     0, 1,16,  TRUE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_DISP16",    FALSE, 0, 0x0000ffff,TRUE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_DISP26,     2, 2,26,  TRUE,0, complain_overflow_signed,	bfd_elf_generic_reloc, "R_88K_DISP26",    FALSE, 0, 0x03ffffff,TRUE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_PLT_DISP26, 0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_PLT_DISP26",FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_BBASED_32,  0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_BBASED_32", FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_BBASED_32UA,0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_BBASED_32UA",FALSE, 0,0x00000000,FALSE),
  HOWTO(R_88K_BBASED_16H, 0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_BBASED_16H",FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_BBASED_16L, 0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_BBASED_16L",FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_ABDIFF_32,  0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_ABDIFF_32", FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_ABDIFF_32UA,0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_ABDIFF_32UA",FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_ABDIFF_16H, 0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_ABDIFF_16H", FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_ABDIFF_16L, 0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_ABDIFF_16L", FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_ABDIFF_16,  0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_ABDIFF_16",  FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_NONE,       0, 0, 0, FALSE,0, complain_overflow_dont,	bfd_elf_generic_reloc, "R_88K_NONE",      FALSE, 0, 0x00000000,FALSE),
  HOWTO(R_88K_32,         0, 2,32, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_32",        FALSE, 0, 0xffffffff,FALSE),
  HOWTO(R_88K_32UA,       0, 2,32, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_32UA",      FALSE, 0, 0xffffffff,FALSE),
  HOWTO(R_88K_16H,        0, 2,16, FALSE,0, complain_overflow_dont, 	bfd_elf_generic_reloc, "R_88K_16H",       FALSE, 0xffff, 0xffff,FALSE),
  HOWTO(R_88K_16L,        0, 2,16, FALSE,0, complain_overflow_dont, 	bfd_elf_generic_reloc, "R_88K_16L",       FALSE, 0xffff, 0xffff,FALSE),
  HOWTO(R_88K_16,         0, 1,16, FALSE,0, complain_overflow_bitfield,	bfd_elf_generic_reloc, "R_88K_16",        FALSE, 0, 0x0000ffff,FALSE),
};

static const struct
{
  bfd_reloc_code_real_type bfd_val;
  int elf_val;
} reloc_map[] = {
  { BFD_RELOC_NONE, R_88K_NONE },
  { BFD_RELOC_32, R_88K_32 },
  { BFD_RELOC_16, R_88K_16 },
  { BFD_RELOC_8, R_88K_8 },
  { BFD_RELOC_HI16, R_88K_16H},
  { BFD_RELOC_LO16, R_88K_16L},
  { BFD_RELOC_M88K_26_PCREL, R_88K_DISP26},
  { BFD_RELOC_16_PCREL, R_88K_DISP16},
};

static reloc_howto_type *
reloc_type_lookup (abfd, code)
     bfd *abfd ATTRIBUTE_UNUSED;
     bfd_reloc_code_real_type code;
{
  /* looks like if we are given a bfd reloc type we do the wrong thing  
  BFD_ASSERT ( code >= 0 && code <= R_88K_max);
  return &howto_table[code];*/

  unsigned int i;
  for (i = 0; i < sizeof (reloc_map) / sizeof (reloc_map[0]); i++)
    {
      if (reloc_map[i].bfd_val == code)
	return &howto_table[reloc_map[i].elf_val];
    }
  return 0;
}
#define bfd_elf32_bfd_reloc_type_lookup reloc_type_lookup

static void
rtype_to_howto (abfd, cache_ptr, dst)
     bfd *abfd ATTRIBUTE_UNUSED;
     arelent *cache_ptr;
     Elf_Internal_Rela *dst;
{
  BFD_ASSERT (ELF32_R_TYPE(dst->r_info) < (unsigned int) R_88K_max);
  cache_ptr->howto = &howto_table[ELF32_R_TYPE(dst->r_info)];
}

#define elf_info_to_howto      rtype_to_howto

/* m88k ELF linker hash entry.  */

struct elf_m88k_link_hash_entry
{
  struct elf_link_hash_entry root;

  /* I suspect I'll need to put stuff here.  */
};

#define elf_m88k_hash_entry(ent) ((struct elf_m88k_link_hash_entry *) (ent))

/* m88k ELF linker hash table.  */

struct elf_m88k_link_hash_table
{
  struct elf_link_hash_table root;

  /* and here too.  */
};

/* Get the m88k ELF linker hash table from a link_info structure.  */

#define elf_m88k_hash_table(p) \
  ((struct elf_m88k_link_hash_table *) (p)->hash)

/* Create an entry in an m88k ELF linker hash table.  */

static struct bfd_hash_entry *
elf_m88k_link_hash_newfunc (entry, table, string)
     struct bfd_hash_entry *entry;
     struct bfd_hash_table *table;
     const char *string;
{
  struct bfd_hash_entry *ret = entry;

  /* Allocate the structure if it has not already been allocated by a
     subclass.  */
  if (ret == NULL)
    ret = bfd_hash_allocate (table,
			     sizeof (struct elf_m88k_link_hash_entry));
  if (ret == NULL)
    return ret;

  /* Call the allocation method of the superclass.  */
  ret = _bfd_elf_link_hash_newfunc (ret, table, string);
  if (ret != NULL)
    /*elf_m88k_hash_entry (ret)->pcrel_relocs_copied = NULL; */
    1;

  return ret;
}

/* Create an m88k ELF linker hash table.  */

static struct bfd_link_hash_table *
elf_m88k_link_hash_table_create (abfd)
     bfd *abfd;
{
  struct elf_m88k_link_hash_table *ret;
  bfd_size_type amt = sizeof (struct elf_m88k_link_hash_table);

  ret = (struct elf_m88k_link_hash_table *) bfd_malloc (amt);
  if (ret == (struct elf_m88k_link_hash_table *) NULL)
    return NULL;

  if (! _bfd_elf_link_hash_table_init (&ret->root, abfd,
				       elf_m88k_link_hash_newfunc))
    {
      free (ret);
      return NULL;
    }

  /*ret->sym_sec.abfd = NULL;*/

  return &ret->root.root;
}

/* Relocate an M88K ELF section.  */

static bfd_boolean
elf_m88k_relocate_section (output_bfd, info, input_bfd, input_section,
			   contents, relocs, local_syms, local_sections)
     bfd *output_bfd;
     struct bfd_link_info *info;
     bfd *input_bfd;
     asection *input_section;
     bfd_byte *contents;
     Elf_Internal_Rela *relocs;
     Elf_Internal_Sym *local_syms;
     asection **local_sections;
{
  struct elf_link_hash_entry **sym_hashes;
  Elf_Internal_Shdr *symtab_hdr;
  Elf_Internal_Rela *rel;
  Elf_Internal_Rela *relend;

  sym_hashes = elf_sym_hashes (input_bfd);
  symtab_hdr = &elf_tdata (input_bfd)->symtab_hdr;

  rel = relocs;
  relend = relocs + input_section->reloc_count;
  for (; rel < relend; rel++)
    {
      int r_type;
      reloc_howto_type *howto;
      unsigned long r_symndx;
      struct elf_link_hash_entry *h;
      Elf_Internal_Sym *sym;
      asection *sec;

      r_type = ELF32_R_TYPE (rel->r_info);
      bfd_vma relocation;
      bfd_boolean unresolved_reloc;
      bfd_reloc_status_type r;
      howto = howto_table + r_type;
      r_symndx = ELF32_R_SYM (rel->r_info);

#if 1 /* TKM XXX - Stolen from elf64-alpha.c.  Not sure if this is correct, but it looks reasonable. -TKM */
      if (info->relocatable)
	{
	  /* This is a relocatable link.  We don't have to change
	     anything, unless the reloc is against a section symbol,
	     in which case we have to adjust according to where the
	     section symbol winds up in the output section.  */
	  if (r_symndx < symtab_hdr->sh_info)
	    {
	      sym = local_syms + r_symndx;
	      if (ELF_ST_TYPE(sym->st_info) == STT_SECTION)
		{
		  sec = local_sections[r_symndx];
		  rel->r_addend += sec->output_offset + sym->st_value;
		}
	    }
	    continue;
	}
#endif

      h = NULL;
      sym = NULL;
      sec = NULL;
      unresolved_reloc = FALSE;

      if (r_symndx < symtab_hdr->sh_info)
	{
	  sym = local_syms + r_symndx;
	  sec = local_sections[r_symndx];
	  relocation = _bfd_elf_rela_local_sym (output_bfd, sym, &sec, rel);
	}
      else
	{
	  bfd_boolean warned;

	  RELOC_FOR_GLOBAL_SYMBOL (info, input_bfd, input_section, rel,
				   r_symndx, symtab_hdr, sym_hashes,
				   h, sec, relocation,
				   unresolved_reloc, warned);
	}

      switch (r_type)
	{
	case R_88K_32:
	  1;
	}

      if (r_type == R_88K_16H)
	r = elf_m88k_relocate_hi16(input_bfd, rel,
				     contents, relocation);
      else if (r_type == R_88K_16L)
	r = elf_m88k_relocate_lo16(input_bfd, rel,
				     contents, relocation);
      else
	r = _bfd_final_link_relocate (howto, input_bfd, input_section,
				      contents, rel->r_offset,
				      relocation, rel->r_addend);
      if (r != bfd_reloc_ok)
	{
	  const char *name;

	  if (h != NULL)
	    name = h->root.root.string;
	  else
	    {
	      name = bfd_elf_string_from_elf_section (input_bfd,
						      symtab_hdr->sh_link,
						      sym->st_name);
	      if (name == NULL)
		return FALSE;
	      if (*name == '\0')
		name = bfd_section_name (input_bfd, sec);
	    }

	  if (r == bfd_reloc_overflow)
	    {
	      if (!(info->callbacks->reloc_overflow
		    (info, name, howto->name, (bfd_vma) 0,
		     input_bfd, input_section, rel->r_offset)))
		return FALSE;
	    }
	  else
	    {
	      (*_bfd_error_handler)
		(_("%s(%s+0x%lx): reloc against `%s': error %d"),
		 bfd_archive_filename (input_bfd),
		 bfd_get_section_name (input_bfd, input_section),
		 (long) rel->r_offset, name, (int) r);
	      return FALSE;
	    }
	}
    }

  return TRUE;
}

/* Handle an m88k ELF HI16/LO16 relocs.  Taken from FRV */

static bfd_reloc_status_type
elf_m88k_relocate_hi16 (input_bfd, relhi, contents, value)
     bfd *input_bfd;
     Elf_Internal_Rela *relhi;
     bfd_byte *contents;
     bfd_vma value;
{
  value += relhi->r_addend;
  value = ((value >> 16) & 0xffff);

  if ((long) value > 0xffff || (long) value < -0x10000)
    return bfd_reloc_overflow;

  bfd_put_16 (input_bfd, value, contents + relhi->r_offset);
  return bfd_reloc_ok;
}

static bfd_reloc_status_type
elf_m88k_relocate_lo16 (input_bfd, rello, contents, value)
     bfd *input_bfd;
     Elf_Internal_Rela *rello;
     bfd_byte *contents;
     bfd_vma value;
{
  value += rello->r_addend;
  value = value & 0xffff;

  if ((long) value > 0xffff || (long) value < -0x10000)
    return bfd_reloc_overflow;

  bfd_put_16 (input_bfd, value, contents + rello->r_offset);
  return bfd_reloc_ok;
}

#define TARGET_BIG_SYM		bfd_elf32_m88k_vec
#define TARGET_BIG_NAME		"elf32-m88k"
#define ELF_ARCH		bfd_arch_m88k
#define ELF_MACHINE_CODE	EM_88K
#define ELF_MAXPAGESIZE  	0x10000 /* the max page size in bytes.  */

#define bfd_elf32_bfd_link_hash_table_create \
					elf_m88k_link_hash_table_create
#define elf_backend_relocate_section	elf_m88k_relocate_section


#include "elf32-target.h"
