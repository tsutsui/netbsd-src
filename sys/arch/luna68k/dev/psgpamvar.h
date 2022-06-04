/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 */
#define XPENC_PCM1	0
#define XPENC_PCM2	1
#define XPENC_PCM3	2

#define XPENC_PAM2_0	3
#define XPENC_PAM2_9	4
#define XPENC_PAM2_12	5
#define XPENC_PAM2_19	6

#define XPENC_PAM3_0	7
#define XPENC_PAM3_12	8

#define XPENC_PAM4_12	9

#define XPENC_MAX	10

#define XP_VAR_BASE     0x0100
#define XP_MAGIC        (XP_VAR_BASE + 0)

#define XP_CMD_START    (XP_VAR_BASE + 8)
#define XP_TIMER        (XP_VAR_BASE + 9)
#define XP_ENC          (XP_VAR_BASE + 10)

/* word */
#define XP_BUFTOP       (XP_VAR_BASE + 12)

#define XP_STAT_READY   (XP_VAR_BASE + 14)
#define XP_STAT_ERROR   (XP_VAR_BASE + 15)
#define XP_STAT_PLAY    (XP_VAR_BASE + 16)
#define XP_STAT_RESET   (XP_VAR_BASE + 17)
/* word */
#define XP_STAT_PTR     (XP_VAR_BASE + 18)

#define XP_CPU_FREQ	6144000
