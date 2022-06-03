#if !defined(_KERNEL)
#include <stdint.h>
#include <unistd.h>
#endif

uint8_t builtin_xplx[] = {
#include "xplx/xplx.inc"
};
ssize_t xplx_size = sizeof(builtin_xplx);
uint8_t *xplx = builtin_xplx;
