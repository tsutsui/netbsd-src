/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 * - consider how xplx.inc should be handled
 * - make builtin_xplx[] and xplx const?
 * - ssize_t or size_t?
 * - KNF
 */
#if !defined(_KERNEL)
#include <stdint.h>
#include <unistd.h>
#endif

uint8_t builtin_xplx[] = {
#include "xplx/xplx.inc"
};
ssize_t xplx_size = sizeof(builtin_xplx);
uint8_t *xplx = builtin_xplx;
