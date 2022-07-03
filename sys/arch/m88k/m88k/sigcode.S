#if 0	/* XXX - believe this is only needed for compat. -TKM */
/*
 * Signal trampoline code.
 * The kernel arranges for the handler to be invoked directly, and return
 * here.
 */
GLOBAL(sigcode)			/* r31 points to sigframe */
	ld	r2,  r31, 0	/* pick sigcontext* */
	or	r13, r0,  SYS_sigreturn
	tb0	0,   r0,  128	/* syscall trap, calling sigreturn */
	NOP			| failure return
#ifdef dontbother		/* sigreturn will not return unless it fails */
	NOP			| success return
#endif
	or	r13, r0,  SYS_exit
	tb0	0,   r0,  128	/* syscall trap, exit */
	/*
	 * this never returns, but we need to provide fetchable instructions
	 * for the 88100 pipeline.
	 */
	NOP
	NOP
GLOBAL(esigcode)
#endif
