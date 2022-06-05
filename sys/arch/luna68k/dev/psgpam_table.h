/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 */
#ifndef _PSGPAM_TABLE_H_
#define _PSGPAM_TABLE_H_

#define PCM1_TABLE_BITS 	8
#define PCM2_TABLE_BITS 	8
#define PCM3_TABLE_BITS 	9
#define PAM2A_TABLE_BITS	8
#define PAM2B_TABLE_BITS	8
#define PAM3A_TABLE_BITS	11
#define PAM3B_TABLE_BITS	9

extern const uint8_t  PCM1_TABLE[];
extern const uint8_t  PCM2_TABLE[];
extern const uint16_t PCM3_TABLE[];
extern const uint8_t  PAM2A_TABLE[];
extern const uint8_t  PAM2B_TABLE[];
extern const uint16_t PAM3A_TABLE[];
extern const uint16_t PAM3B_TABLE[];
#endif /* !_PSGPAM_TABLE_H_ */
