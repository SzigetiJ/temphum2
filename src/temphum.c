/*
 * Copyright 2026 SZIGETI János
 *
 * This file is part of TempHum2 application, which is released under GNU General Public License.version 3.
 * See LICENSE or <https://www.gnu.org/licenses/> for full license details.
 */
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "dht22.h"
#include "esp_attr.h"
#include "gpio.h"
#include "main.h"
#include "rmt.h"
#include "defines.h"
#include "romfunctions.h"
#include "iomux.h"
#include "dport.h"
#include "timg.h"
#include "tm1637.h"
#include "typeaux.h"
#include "utils/uartutils.h"
#include "i16utils.h"

// =================== Hard constants =================
// #1: Timings
#define RMTDHT_PERIOD_MS      2000U
#define DISPLAY_DELAY_MS       100U
#define DISPLAY_PERIOD_MS     1000U
#define UART_FREQ_HZ        115200U
#define ALIVE_BLINK_PERIOD_MS 5000U
#define ALIVE_BLINK_ON_MS       50U
#define MEASLOG_IDLE_PERIOD_MS 500U
#define MEASLOG_PERIOD_MS       10U

// #2: Channels / wires / addresses
#define LED0_GPIO                2U
#define LED1_GPIO               17U
#define BUTTON0_GPIO            18U
#define BUTTON1_GPIO            18U // not connected
#define BUTTON2_GPIO             0U // onboard button - used for TM1637 brightness cycling
#define TM1637CLK_GPIO          25U
#define TM1637DIO_GPIO          26U
#define DHT22_GPIO              27U

#define DHT22_RMTCH         RMT_CH0
#define TM1637CLK_RMTCH     RMT_CH1
#define TM1637DIO_RMTCH     RMT_CH2
#define RMTINT_CH               23U
#define BUTTONINT_CH            22U

// #3: Sizes
#define TM1637_CELLS             4U
#define TM1637_COLON_POS         1U

// #4 Default values
#define TEMPSTORE_BASE 200
#define RHUMSTORE_BASE 500

// ============= Local types ===============

typedef enum {
  TM1637_INIT = 0,
  TM1637_REGULAR,
  TM1637_MINMAX
} E_TM1637_MAJOR_STATE;

typedef enum {
  TM1637REG_TEMP = 0,
  TM1637REG_HUM
} E_TM1637_REGULAR_STATE;

typedef enum {
  TM1637MM_ANNOUNCETEMP = 0,
  TM1637MM_TEMPMIN,
  TM1637MM_TEMPMAX,
  TM1637MM_ANNOUNCERHUM,
  TM1637MM_RHUMMIN,
  TM1637MM_RHUMMAX,
  TM1637MM_NIL
} E_TM1637_MINMAX_STATE;

typedef struct {
  int16_t min;
  int16_t max;
  int16_t cur;
  bool bValid;
} S_MINMAXCUR;

typedef struct {
  STm1637State *psState;
  uint64_t u64tckStart;
} SReadyCbParam;


// ================ Local function declarations =================
void _dht22_run_ready_cb(void *pvParam, SDht22Data *psParam);
static void _alive_blink_init();
static void _alive_blink_cycle(uint64_t u64tckNow);

static inline SGpioPinReg _gpio_pinreg(uint32_t u1PadDriver, uint32_t u3PinIntType, uint32_t u1WakeUpEn, uint32_t u5PinIntEn);
static inline IomuxGpioConfReg _iomux_gpioconfreg(uint32_t u3McuSel, uint32_t u2FunDrv, uint32_t u1FunIE,
        uint32_t u2FunWPUD, uint32_t u2McuDrv, uint32_t u1McuIE, uint32_t u2McuWPUD, uint32_t u1SlpSel, uint32_t u1McuOE);
static void _configure_button(uint8_t u8Gpio);
static void _button_init();
static void _button_isr(void *pvParam);

static void _display_init();
static void _display_cycle(uint64_t u64tckNow);
static void _display_ready(void *pvParam);
static void _asciiseq_to_seg7(uint8_t *pu8Dst, const char *pcSrc, uint8_t u8Len);
static void _num_to_asciiseq(char *pcDst, uint16_t u16Num, uint8_t u8Len);
static void _i16_to_asciiseq(char *pcDst, int16_t i16Value, bool bTemp);

static void _dht22_init();
static void _dht22_cycle(uint64_t u64tckNow);

static void _measproc_init();
static void _measproc_cycle(uint64_t u64tckNow);


// =================== Global constants ================
const bool gbStartAppCpu = START_APP_CPU;
const uint16_t gu16Tim00Divisor = TIM0_0_DIVISOR;
const uint64_t gu64tckSchedulePeriod = (CLK_FREQ_HZ / SCHEDULE_FREQ_HZ);

// ==================== Local Data ================
static const TimerId gsTimer = {.eTimg = TIMG_0, .eTimer = TIMER0};

// display data
const uint8_t gau8NumToSeg[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};

static uint8_t gau8Tm1637Data[TM1637_CELLS];
static bool gbTm1637FullFlushRequired = true;
static STm1637State gsTm1637State;
static SReadyCbParam gsReadyData;

// what is displayed - states
static E_TM1637_MAJOR_STATE geDisplayMajorState = TM1637_INIT;
static E_TM1637_MINMAX_STATE geDisplayMMState;

// T/RH measurement data
static SDht22Descriptor gsDht22Desc;
DRAM_ATTR static volatile bool gbDht22ResultReady = false;

// all-time stats
static StatStore gsTemp;
static StatStore gsRhum;

// stats per min/hour/day
static uint32_t gu32StoreMinValidMask = 0;
static int16_t gai16TempStoreMin[30];
static int16_t gai16RhumStoreMin[30];
static uint8_t gu8StoreMinIdx = -1;
static StatStore gasTempStoreHour[60];
static StatStore gasRhumStoreHour[60];
static uint8_t gu8StoreHourIdx = -1;
// TODO: for each hour store some statistics

// logging is done in multiple cycles. Therefore we need some static counters
static bool gbMeasLog = false;
static uint8_t gu8MeasLogMinIdx = 0;
static uint8_t gu8MeasLogHourIdx = 0;
static uint8_t gu8MeasLogDayIdx = 0;

// debug variables
static bool gbDebugDHT22 = false;

// ============== Implementation ==============
// -------------- Internal functions --------------

static inline SGpioPinReg _gpio_pinreg(uint32_t u1PadDriver, uint32_t u3PinIntType, uint32_t u1WakeUpEn, uint32_t u5PinIntEn) {
  return (SGpioPinReg)((u1PadDriver << 2) | (u3PinIntType << 7) | (u1WakeUpEn << 10) | (u5PinIntEn << 13));
}

static inline IomuxGpioConfReg _iomux_gpioconfreg(uint32_t u3McuSel, uint32_t u2FunDrv, uint32_t u1FunIE,
        uint32_t u2FunWPUD, uint32_t u2McuDrv, uint32_t u1McuIE, uint32_t u2McuWPUD, uint32_t u1SlpSel, uint32_t u1McuOE) {
  return (IomuxGpioConfReg) ((u3McuSel << 12) | (u2FunDrv << 10) | (u1FunIE << 9) | (u2FunWPUD << 7) | (u2McuDrv << 5) | (u1McuIE << 4)
          | (u2McuWPUD << 2) | (u1SlpSel << 1) | (u1McuOE << 0));
}

///////////////////////////
// ALIVE BLINKING section

static void _alive_blink_init() {
  gpio_pin_enable(LED0_GPIO);
}

static void _alive_blink_cycle(uint64_t u64tckNow) {
  static uint64_t u64tckNext = 0;
  static bool bState = false;
  if (u64tckNext <= u64tckNow) {
    bState = !bState;
    gpio_reg_setbit(bState ? &gsGPIO.OUT_W1TS : &gsGPIO.OUT_W1TC, LED0_GPIO);
    u64tckNext += MS2TICKS(bState ? ALIVE_BLINK_ON_MS : (ALIVE_BLINK_PERIOD_MS - ALIVE_BLINK_ON_MS));
  }
}

///////////////////////////
// BUTTON section

IRAM_ATTR static void _button_isr(void *pvParam) {
  bool bButton0Event = gsGPIO.STATUS & (1 << BUTTON0_GPIO);
  bool bButton2Event = gsGPIO.STATUS & (1 << BUTTON2_GPIO);
  if (bButton0Event) {
    gsGPIO.STATUS_W1TC = (1 << BUTTON0_GPIO);
    bool bLevel = gsGPIO.IN & (1 << BUTTON0_GPIO);
    gsUART0.FIFO = bLevel ? '^' : '_';
    if (!bLevel) {
      geDisplayMMState = TM1637MM_ANNOUNCETEMP;
      geDisplayMajorState = TM1637_MINMAX;
    }
  }

  if (bButton2Event) {
    gsGPIO.STATUS_W1TC = (1 << BUTTON2_GPIO);
    bool bLevel = gsGPIO.IN & (1 << BUTTON2_GPIO);
    gsUART0.FIFO = bLevel ? '*' : 'x';
    if (!bLevel) {
      uint8_t u8Curr = gsTm1637State.u8Brightness & 7;
      ++u8Curr;
      gsTm1637State.u8Brightness = 0x08 | (u8Curr & 7);
      gbTm1637FullFlushRequired = true;
      gbMeasLog = true;
    }
  }
}

static void _configure_button(uint8_t u8Gpio) {
  SGpioPinReg rGpioPinN = _gpio_pinreg(0, 3, 1, 4);
  IomuxGpioConfReg rIOMuxX = _iomux_gpioconfreg(2, 0, 1, 2, 0, 1, 2, 0, 0);

  iomux_set_gpioconf(u8Gpio, rIOMuxX);
  gsGPIO.PIN[u8Gpio] = rGpioPinN;
  gsGPIO.FUNC_OUT_SEL_CFG[u8Gpio] = (1 << 10) | 256;
  gpio_pin_disable(u8Gpio);
  gpio_reg_setbit(&gsGPIO.STATUS_W1TC, u8Gpio);
}

static void _button_init() {
  // setup iomux & gpio regs
  _configure_button(BUTTON0_GPIO);
  _configure_button(BUTTON2_GPIO);

  // register ISR and enable it
  ECpu eCpu = CPU_PRO;
  RegAddr prDportIntMap = (eCpu == CPU_PRO ? &dport_regs()->PRO_GPIO_INTERRUPT_MAP : &dport_regs()->APP_GPIO_INTERRUPT_MAP);

  *prDportIntMap = BUTTONINT_CH;
  _xtos_set_interrupt_handler_arg(BUTTONINT_CH, _button_isr, 0);
  ets_isr_unmask(1 << BUTTONINT_CH);
}

///////////////////////////
// DISPLAY section

static void _display_ready(void *pvParam) {
  // nothing to do
}

static void _display_cycle(uint64_t u64tckNow) {
  static uint64_t u64NextTick = MS2TICKS(DISPLAY_DELAY_MS);
  static E_TM1637_REGULAR_STATE eRegState = TM1637REG_TEMP;
  static int8_t i8InitScrollOffset = -3;
  char acDispData[TM1637_CELLS];

  if (u64NextTick <= u64tckNow) {
    switch (geDisplayMajorState) {
      case TM1637_INIT: // scroll nums right to left
        for (int i = 0; i < TM1637_CELLS; ++i) {
          int idx = i + i8InitScrollOffset;
          gau8Tm1637Data[i] = ((idx < 0 || ARRAY_SIZE(gau8NumToSeg) <= idx) ? 0 : gau8NumToSeg[idx]);
        }
        ++i8InitScrollOffset;
        if (18 < i8InitScrollOffset) {
          geDisplayMajorState = TM1637_REGULAR;
        }
        break;
      case TM1637_MINMAX:
        if (geDisplayMMState == TM1637MM_TEMPMIN) {
          gbTm1637FullFlushRequired = true;
        }
        bool bAnnounce = (geDisplayMMState % 3 == 0);
        bool bTemp = (geDisplayMMState < 3);
        bool bMax = (geDisplayMMState % 3 == 2);
        if (bAnnounce) {
          for (int i = 0; i < TM1637_CELLS; ++i) gau8Tm1637Data[i] = 0;
          gau8Tm1637Data[TM1637_COLON_POS] = (bTemp ? 0xf8 : 0xf4); // "t:" vs. "h:"
        } else {
          StatStore *psX = bTemp ? &gsTemp : &gsRhum;
          int16_t i16Val = bMax ? psX->ai16MaxDat[1] : psX->ai16MinDat[1];
          _i16_to_asciiseq(acDispData, i16Val, bTemp);
          if (psX->i32Cnt < 2) {
            for (int i = 0; i < ARRAY_SIZE(acDispData); ++i) acDispData[i] = '-';
          }
          _asciiseq_to_seg7(gau8Tm1637Data, acDispData, 4);
        }
        ++geDisplayMMState;
        if (geDisplayMMState == TM1637MM_NIL)
          geDisplayMajorState = TM1637_REGULAR;
        break;
      case TM1637_REGULAR:
      {
        bool bTemp = (eRegState == TM1637REG_TEMP);
        int16_t i16Value = bTemp ? gai16TempStoreMin[gu8StoreMinIdx] : gai16RhumStoreMin[gu8StoreMinIdx];
        _i16_to_asciiseq(acDispData, i16Value, bTemp);
        _asciiseq_to_seg7(gau8Tm1637Data, acDispData, 4);
        eRegState = (eRegState == TM1637REG_TEMP) ? TM1637REG_HUM : TM1637REG_TEMP;
      }

        break;
    }
    gsReadyData.u64tckStart = timg_ticks(gsTimer);
    if (gbTm1637FullFlushRequired) {
      tm1637_flush_full(&gsTm1637State, TM1637_CELLS);
      gbTm1637FullFlushRequired = false;
    } else {
      tm1637_flush_range(&gsTm1637State, 0, TM1637_CELLS);
    }
    u64NextTick += MS2TICKS(geDisplayMajorState == TM1637_INIT ? 200 : DISPLAY_PERIOD_MS);
  }
}

static void _display_init() {
  STm1637Iface sIface = { TM1637CLK_GPIO, TM1637DIO_GPIO, TM1637CLK_RMTCH, TM1637DIO_RMTCH};
  gsTm1637State = tm1637_config(&sIface, gau8Tm1637Data);
  tm1637_init(&gsTm1637State, APB_FREQ_HZ);
  tm1637_set_readycb(&gsTm1637State, _display_ready, &gsReadyData);
  tm1637_set_brightness(&gsTm1637State, true, 7);
  gsReadyData.psState = &gsTm1637State;
}

/**
 * Converts a sequence of ascii characters to sequence of 7segments display symbols.
 * Currently, only hexadecimal characters are supported (0-9, A-F, a-f) and '-' sign.
 * Any other character is converted into space.
 * @param pu8Dst Reference of output sequence.
 * @param pcSrc Reference of input sequence.
 * @param u8Len Length of the input sequence to convert.
 */
static void _asciiseq_to_seg7(uint8_t *pu8Dst, const char *pcSrc, uint8_t u8Len) {
  for (uint8_t i = 0; i < u8Len; ++i) {
    char cSrc = pcSrc[i];
    pu8Dst[i] = ('0' <= cSrc && cSrc <= '9') ? gau8NumToSeg[cSrc - '0'] :
            ('A' <= cSrc && cSrc <= 'F') ? gau8NumToSeg[cSrc - 'A' + 10] :
            ('a' <= cSrc && cSrc <= 'f') ? gau8NumToSeg[cSrc - 'a' + 10] :
            ('-' == cSrc) ? 0x40 :
            0;
  }
}

/**
 * Converts number to decimal number sequence.
 * Processes only the last u8Len digits of the input number.
 * @param pu8Dst Ref of output store.
 * @param u16Num Input number.
 * @param u8Len Number of digits to generate.
 */
static void _num_to_asciiseq(char *pcDst, uint16_t u16Num, uint8_t u8Len) {
  for (uint8_t i = 0; i < u8Len; ++i) {
    pcDst[u8Len - i - 1] = '0' + (u16Num % 10);
    u16Num /= 10;
  }
}

/**
 *  Complex write a value to 4x7 segment display.
 *  Complexity:
 *   -- if (!bTemp) last cell is empty, otherwise last cell displays 'C'.
 *   -- that means that the first 3 cells display positive values [000..999]
 *   -- if the leading displayed value is '0', it is not displayed.
 *   -- if the value is negative, the displayed characters are SHR by 1, and '-' is put in front of the first not empty cell.
 *  what to display?
 *  12.3°C -> "123C"
 *   8.2°C -> " 82C"
 *   0.4°C -> " 04C"
 *  56.4%  -> "564 "
 *   9.9%  -> " 99 "
 * -10.2°C -> "-102" (A) OR "102-" (B)?
 *  -4.5°C -> " -45" (A) OR "-45C" (B)?
 *  -0.6°C -> " -06" (A) OR "-06C" (B)?
 * Go with variant (A)!
 * @param pu8DispSegs 4 byte wide result store.
 * @param i16Value the last 3 digits of this value get displayed.
 * @param bTemp true: display temperature, false: display humidity.
 */
static void _i16_to_asciiseq(char *pcDst, int16_t i16Value, bool bTemp) {
  bool bNegative = i16Value < 0;
  uint16_t u16ValueAbs = bNegative ? -i16Value : i16Value;
  bool bZeroTens = (u16ValueAbs < 100);
  bool bSuffixC = !bNegative && bTemp;
  uint8_t u8NegSignPos = bZeroTens ? 1 : 0;

  pcDst[3] = bSuffixC ? 'C' : ' ';  // conditionally overwritten by the following codeline
  _num_to_asciiseq(pcDst + (bNegative ? 1 : 0), u16ValueAbs, 3);
  if (bZeroTens) {
    pcDst[0] = ' ';
  }
  if (bNegative) {
    pcDst[u8NegSignPos] = '-';
  }
}

static void _measlog_cycle(uint64_t u64tckNow) {
  static uint64_t u64tckNext = MS2TICKS(MEASLOG_IDLE_PERIOD_MS);
  if (u64tckNext <= u64tckNow) {
    if (gbMeasLog) {
      if (gu8MeasLogDayIdx < 24) {
        // log last day
        ++gu8MeasLogDayIdx;
      } else if (gu8MeasLogHourIdx < 60) {
        // log last hour
        if (gu8MeasLogHourIdx == 0) {
          uart_printf(&gsUART0, "*** LAST Hour ***\r\n");
        }
        if (0 < gasTempStoreHour[gu8MeasLogHourIdx].i32Cnt) {
          uart_printf(&gsUART0, "#%u%c\t%d:\t%4d -- %4d\t(%d),\t%4d -- %4d\t(%d)\r\n", gu8MeasLogHourIdx, gu8MeasLogHourIdx == gu8StoreHourIdx ? '*' : ' ',
                  gasTempStoreHour[gu8MeasLogHourIdx].i32Cnt,
                  gasTempStoreHour[gu8MeasLogHourIdx].ai16MinDat[1], gasTempStoreHour[gu8MeasLogHourIdx].ai16MaxDat[1],
                  2 < gasTempStoreHour[gu8MeasLogHourIdx].i32Cnt ? statstore_avg_cut(&gasTempStoreHour[gu8MeasLogHourIdx]) : 0,
                  gasRhumStoreHour[gu8MeasLogHourIdx].ai16MinDat[1], gasRhumStoreHour[gu8MeasLogHourIdx].ai16MaxDat[1],
                  2 < gasRhumStoreHour[gu8MeasLogHourIdx].i32Cnt ? statstore_avg_cut(&gasRhumStoreHour[gu8MeasLogHourIdx]) : 0);
        } else {
          uart_printf(&gsUART0, "#%u%c0:\t---\r\n", gu8MeasLogHourIdx, gu8MeasLogHourIdx == gu8StoreHourIdx ? '*' : ' ');
        }
        ++gu8MeasLogHourIdx;
      } else if (gu8MeasLogMinIdx < 30) {
        // log last minute
        if (gu8MeasLogMinIdx == 0) {
          uart_printf(&gsUART0, "*** LAST Minute ***\r\n");
        }
        if (gu32StoreMinValidMask & (1 << gu8MeasLogMinIdx)) {
          uart_printf(&gsUART0, "#%u%c\t%d\t\t%d\r\n", gu8MeasLogMinIdx, gu8MeasLogMinIdx == gu8StoreMinIdx ? '*' : ' ',
                  gai16TempStoreMin[gu8MeasLogMinIdx], gai16RhumStoreMin[gu8MeasLogMinIdx]);
        } else {
          uart_printf(&gsUART0, "#%u%c\t---\t\t---\r\n", gu8MeasLogMinIdx, gu8MeasLogMinIdx == gu8StoreMinIdx ? '*' : ' ');
        }
        ++gu8MeasLogMinIdx;
      } else {
        // ready
        gbMeasLog = false;
        gu8MeasLogDayIdx = 0;
        gu8MeasLogHourIdx = 0;
        gu8MeasLogMinIdx = 0;
        // print all-time min/max
        uart_printf(&gsUART0, "all-time: %d, %4d -- %4d (%d), %4d -- %4d (%d)\r\n", gsTemp.i32Cnt,
                gsTemp.ai16MinDat[1], gsTemp.ai16MaxDat[1],
                2 < gsTemp.i32Cnt ? statstore_avg_cut(&gsTemp) : 0,
                gsRhum.ai16MinDat[1], gsRhum.ai16MaxDat[1],
                2 < gsRhum.i32Cnt ? statstore_avg_cut(&gsRhum) : 0);
      }
    }
    u64tckNext += MS2TICKS(MEASLOG_PERIOD_MS);
  }
}

static void _measproc_init() {
  gsTemp = statstore_init(TEMPSTORE_BASE);
  gsRhum = statstore_init(RHUMSTORE_BASE);
}

static void _measproc_cycle(uint64_t u64tckNow) {
  if (gbDht22ResultReady) {
    SDht22Data *psParam = &gsDht22Desc.sData;
    int16_t i16Temp = dht22_get_temp(psParam);
    int16_t i16Rhum = dht22_get_rhum(psParam);
    bool bValid = dht22_data_valid(psParam);
    // FIX: undocumented error code
    if (i16Temp == 255 && i16Rhum == 255) bValid = false;
    if (gbDebugDHT22) {
      if (!bValid) {
        uart_printf(&gsUART0, "INVALID: %02X %02X %02X %02X %02X\r\n",
                psParam->au8Invalid[0],
                psParam->au8Invalid[1],
                psParam->au8Invalid[2],
                psParam->au8Invalid[3],
                psParam->au8Invalid[4]
                );
      }
      uart_printf(&gsUART0, "DATA: %02X %02X %02X %02X %02X\r\n",
              psParam->au8Data[0],
              psParam->au8Data[1],
              psParam->au8Data[2],
              psParam->au8Data[3],
              psParam->au8Data[4]
              );
      uart_printf(&gsUART0, "raw data (%c) T: %d, RH: %u\r\n",
              bValid ? '+' : '-', i16Temp, i16Rhum
              );
    }
    // stepping the indices
    ++gu8StoreMinIdx;
    if (gu8StoreMinIdx == ARRAY_SIZE(gai16TempStoreMin)) {  // entering new minute
      gu8StoreMinIdx = 0;
      ++gu8StoreHourIdx;
      if (gu8StoreHourIdx == ARRAY_SIZE(gasTempStoreHour)) {
        gsUART0.FIFO = 'H';
        gu8StoreHourIdx = 0;
        // TODO also enter new hour
      }
      uart_printf(&gsUART0, "%02u:%02u:00\r\n", 0, gu8StoreHourIdx);
      gasTempStoreHour[gu8StoreHourIdx] = statstore_init(TEMPSTORE_BASE);
      gasRhumStoreHour[gu8StoreHourIdx] = statstore_init(RHUMSTORE_BASE);
    }

    // store item before overwriting it
    bool bLastValid = gu32StoreMinValidMask & (1 << gu8StoreMinIdx);
    if (bLastValid) {
      statstore_insert(&gasTempStoreHour[gu8StoreHourIdx], gai16TempStoreMin[gu8StoreMinIdx]);
      statstore_insert(&gasRhumStoreHour[gu8StoreHourIdx], gai16RhumStoreMin[gu8StoreMinIdx]);
    }
    if (bValid) {
      gu32StoreMinValidMask |= 1 << gu8StoreMinIdx;
      gai16TempStoreMin[gu8StoreMinIdx] = i16Temp;
      gai16RhumStoreMin[gu8StoreMinIdx] = i16Rhum;
      statstore_insert(&gsTemp, i16Temp);
      statstore_insert(&gsRhum, i16Rhum);
    } else {
      gu32StoreMinValidMask &= ~(1 << gu8StoreMinIdx);
    }
    gbDht22ResultReady = false;
  }
}

void _dht22_run_ready_cb(void *pvParam, SDht22Data *psParam) {
  gbDht22ResultReady = true;
  gpio_pin_out_off(LED1_GPIO);

}

static void _dht22_init() {
  gsDht22Desc = dht22_config(DHT22_RMTCH, _dht22_run_ready_cb, NULL);
  dht22_init(DHT22_GPIO, APB_FREQ_HZ, &gsDht22Desc);
  gpio_pin_enable(LED1_GPIO);
}

static void _dht22_cycle(uint64_t u64Ticks) {
  static uint64_t u64NextTick = 0;

  if (u64NextTick <= u64Ticks) {
    gpio_pin_out_on(LED1_GPIO);
    dht22_run(&gsDht22Desc);

    u64NextTick += MS2TICKS(RMTDHT_PERIOD_MS);
  }
}

// -------------- Interface functions --------------

void prog_init_pro_pre() {
  gsUART0.CLKDIV.raw = UART_HZ2CLKDIV(UART_FREQ_HZ, APB_FREQ_HZ);

  rmt_isr_init();
  rmt_init_controller(true, true);
  rmt_isr_start(CPU_PRO, RMTINT_CH);

  _alive_blink_init();
  _display_init();
  _dht22_init();
  _button_init();
  _measproc_init();


}

void prog_init_app() {
}

void prog_init_pro_post() {
}

void prog_cycle_app(uint64_t u64tckNow) {
}

void prog_cycle_pro(uint64_t u64tckNow) {
  _measproc_cycle(u64tckNow);
  _measlog_cycle(u64tckNow);
  _alive_blink_cycle(u64tckNow);
  _display_cycle(u64tckNow);
  _dht22_cycle(u64tckNow);
}
