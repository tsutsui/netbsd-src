/*	$NetBSD: intel_acpi.h,v 1.5 2024/04/16 14:34:01 riastradh Exp $	*/

/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_ACPI_H__
#define __INTEL_ACPI_H__

#include <linux/acpi.h>

struct drm_i915_private;

#ifdef CONFIG_ACPI
#ifdef __NetBSD__
void intel_register_dsm_handler(struct drm_i915_private *);
#else
void intel_register_dsm_handler(void);
#endif
void intel_unregister_dsm_handler(void);
#else
#ifdef __NetBSD__
static inline void intel_register_dsm_handler(struct drm_i915_private *i915) { return; }
#else
static inline void intel_register_dsm_handler(void) { return; }
#endif
static inline void intel_unregister_dsm_handler(void) { return; }
#endif /* CONFIG_ACPI */

#endif /* __INTEL_ACPI_H__ */
