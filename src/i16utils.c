/*
 * Copyright 2026 SZIGETI János
 *
 * This file is part of TempHum2 application, which is released under GNU General Public License.version 3.
 * See LICENSE or <https://www.gnu.org/licenses/> for full license details.
 */
#include "i16utils.h"
//======== Types ========

//======== Inline function declarations ========
static inline bool _less(int16_t a, int16_t b);
static inline bool _greater(int16_t a, int16_t b);
static inline void _put(int16_t *pi16Dat, int16_t i16Val, bool (*pfLess)(int16_t a, int16_t b));
static inline int16_t _array_extreme(const int16_t *pi16Dat, uint32_t u32Len, bool bMin);
static inline int16_t _array_extreme_masked(const int16_t *pi16Dat, uint32_t u32Len, bool bMin, uint32_t u32Mask);

//======== Implementation ========
//-------- Local functions --------

static inline bool _less(int16_t a, int16_t b) {
  return a<b;
}

static inline bool _greater(int16_t a, int16_t b) {
  return a>b;
}

static inline void _put(int16_t *pi16Dat, int16_t i16Val, bool (*pfLess)(int16_t a, int16_t b)) {
  if (pfLess(i16Val, pi16Dat[0])) {
    pi16Dat[1] = pi16Dat[0];
    pi16Dat[0] = i16Val;
  } else {
    pi16Dat[1] = i16Val;
  }
}

//-------- Interface functions --------

static inline int16_t _array_extreme(const int16_t *pi16Dat, uint32_t u32Len, bool bMin) {
  int16_t i16Ret = pi16Dat[0];
  bool (*pfCmp)(int16_t a, int16_t b) = bMin ? _less : _greater;
  for (uint32_t i = 1; i < u32Len; ++i) {
    if (pfCmp(pi16Dat[i], i16Ret)) {
      i16Ret = pi16Dat[i];
    }
  }
  return i16Ret;
}

static inline int16_t _array_extreme_masked(const int16_t *pi16Dat, uint32_t u32Len, bool bMin, uint32_t u32Mask) {
  int16_t i16Ret = 0;
  bool bValid = false;
  bool (*pfCmp)(int16_t a, int16_t b) = bMin ? _less : _greater;
  for (uint32_t i = 0; i < u32Len && i<32; ++i) {
    if (u32Mask & (1<<i)) {
      if (!bValid || pfCmp(pi16Dat[i], i16Ret)) {
        i16Ret = pi16Dat[i];
        bValid = true;
      }
    }
  }
  return i16Ret;
}

int16_t i16arraymin(const int16_t *pi16Dat, uint32_t u32Len) {
  return _array_extreme(pi16Dat, u32Len, true);
}

int16_t i16arraymax(const int16_t *pi16Dat, uint32_t u32Len) {
  return _array_extreme(pi16Dat, u32Len, false);
}

int16_t i16arraymin_masked(const int16_t *pi16Dat, uint32_t u32Len, uint32_t u32Mask) {
  return _array_extreme_masked(pi16Dat, u32Len, true, u32Mask);
}

int16_t i16arraymax_masked(const int16_t *pi16Dat, uint32_t u32Len, uint32_t u32Mask) {
  return _array_extreme_masked(pi16Dat, u32Len, false, u32Mask);
}

void statstore_insert(StatStore *psStore, int16_t i16Val) {
  if (1 < psStore->i32Cnt) {  // regular branch, min/max dats have 2 items
    if (_less(i16Val, psStore->ai16MinDat[1])) {
      _put(psStore->ai16MinDat, i16Val, _less);
    }
    if (_greater(i16Val, psStore->ai16MaxDat[1])) {
      _put(psStore->ai16MaxDat, i16Val, _greater);
    }
  } else if (psStore->i32Cnt == 0) {  // min/max dats are empty
    psStore->ai16MinDat[0] = i16Val;
    psStore->ai16MaxDat[0] = i16Val;
  } else {  // min/max dats have a single element
    _put(psStore->ai16MinDat, i16Val, _less);
    _put(psStore->ai16MaxDat, i16Val, _greater);
  }
  psStore->i32Sum += i16Val - psStore->i16Base;
  ++psStore->i32Cnt;
}
