/* C fallback for arm_bitreversal_32 (normally arm_bitreversal2.S on CM7).
 * pBitRevTable entries are byte offsets; >>2 converts to uint32_t index.
 * Each complex f32 sample occupies two consecutive uint32_t words (re, im). */
#include "arm_math_types.h"

void arm_bitreversal_32(
        uint32_t * pSrc,
  const uint16_t   bitRevLen,
  const uint16_t * pBitRevTable)
{
    uint32_t a, b, tmp;
    for (uint16_t i = 0; i < bitRevLen; i += 2U)
    {
        a = pBitRevTable[i    ] >> 2;
        b = pBitRevTable[i + 1] >> 2;
        /* swap complex[a] ↔ complex[b]: two consecutive uint32_t words */
        tmp = pSrc[a];     pSrc[a]     = pSrc[b];     pSrc[b]     = tmp;
        tmp = pSrc[a + 1]; pSrc[a + 1] = pSrc[b + 1]; pSrc[b + 1] = tmp;
    }
}
