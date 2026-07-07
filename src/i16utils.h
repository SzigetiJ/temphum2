/*
 * Copyright 2026 SZIGETI János
 *
 * This file is part of TempHum2 application, which is released under GNU General Public License.version 3.
 * See LICENSE or <https://www.gnu.org/licenses/> for full license details.
 */

#ifndef I16UTILS_H
#define I16UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

  //======== Types ========
  typedef struct {
    int16_t ai16MinDat[2];
    int16_t ai16MaxDat[2];
    int32_t i32Sum;
    int32_t i32Cnt;
    int16_t i16Base;
  } StatStore;

  //======== Interface functions ========
  int16_t i16arraymin(const int16_t *pi16Dat, uint32_t u32Len);
  int16_t i16arraymin_masked(const int16_t *pi16Dat, uint32_t u32Len, uint32_t u32Mask);
  int16_t i16arraymax(const int16_t *pi16Dat, uint32_t u32Len);
  int16_t i16arraymax_masked(const int16_t *pi16Dat, uint32_t u32Len, uint32_t u32Mask);
  static inline StatStore statstore_init(int16_t i16Base) {
    return (StatStore){.i32Sum = 0, .i32Cnt = 0, .i16Base = i16Base};
  }

  static inline int32_t statstore_avg(const StatStore *psStore) {
    return psStore->i16Base + (psStore->i32Sum / psStore->i32Cnt);
  }

  static inline int32_t statstore_avg_cut(const StatStore *psStore) {
    int32_t i32Sum = psStore->i32Sum;
    int32_t i32Cnt = psStore->i32Cnt;
    i32Sum -= psStore->ai16MaxDat[0];
    i32Sum -= psStore->ai16MinDat[0];
    i32Sum += 2 * psStore->i16Base;
    i32Cnt -= 2;
    return psStore->i16Base + (i32Sum / i32Cnt);
  }

  void statstore_insert(StatStore *psStore, int16_t i16Val);

#ifdef __cplusplus
}
#endif

#endif /* I16UTILS_H */

