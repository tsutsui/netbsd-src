/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 * - consider how xplx.inc should be handled
 */
#if defined(_KERNEL)
#include <sys/types.h>
#else
#include <stdint.h>
#include <unistd.h>
#endif

static const uint8_t builtin_xplx[] = {
#include "xplx/xplx.inc"
};
const size_t xplx_size = sizeof(builtin_xplx);
const uint8_t *xplx = builtin_xplx;
