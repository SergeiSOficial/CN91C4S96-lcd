/*******************************************************************************
Copyright 2016-2018 anxzhu (github.com/anxzhu)
Copyright 2018 Valerio Nappi (github.com/5N44P) (changes)
Based on segment-lcd-with-ht1621 from anxzhu (2016-2018)
(https://github.com/anxzhu/segment-lcd-with-ht1621)

Partially rewritten and extended by Valerio Nappi (github.com/5N44P) in 2018
https://github.com/5N44P/ht1621-7-seg

Refactored. Removed dependency on any MCU hardware by Viacheslav Balandin
https://github.com/hedgehogV/HT1621-lcd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include "CN91C4S96.h"
#include "main.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief CALCULATION DEFINES BLOCK
 */
#define MAX_NUM 999999999
#define MIN_NUM -999999999
#define MIN_MINUS_NUM -99999999

#define PRECISION_MAX_POSITIVE 5 // TODO: find better names
#define PRECISION_MAX_NEGATIVE 5
#define PRECISION_MIN 1

#define BITS_PER_BYTE 8

#ifndef MIN
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#endif //MIN

#ifndef SET_BIT
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#endif //SET_BIT

#ifndef CLEAR_BIT
#define CLEAR_BIT(REG, BIT) ((REG) |= (BIT))
#endif //CLEAR_BIT

uint8_t BufferSend[DISPLAY_BUFFER_SIZE] = {0}; // Buffer where display data will be stored
uint8_t * Buffer = BufferSend + 2; // Buffer where display data will be stored

uint8_t BufferSendOld[DISPLAY_BUFFER_SIZE] = {0}; // Buffer where display data will be stored

#define LCD_SWITCH(EN, POS, SEG) ((EN) ? (SET_BIT(Buffer[POS], SEG)) : (CLEAR_BIT(Buffer[POS], SEG)))
void LCD_TOGGLE(bool EN, uint8_t POS1, uint8_t SEG1, uint8_t POS2, uint8_t SEG2)
{
    if (EN)
    {
        SET_BIT(Buffer[POS1], SEG1);
        CLEAR_BIT(Buffer[POS2], SEG2);
    }
    else
    {
        SET_BIT(Buffer[POS2], SEG2);
        CLEAR_BIT(Buffer[POS1], SEG1);
    }
}

/**
 * @brief DISPLAY HARDWARE DEFINES BLOCK
 */
#define ULP 0x50 //0b 0101 0000  ULP Set ‘1’ to enable the Ultra-Low-Power mode, which can decrease total power consumption further more along with ‘SR’ and ‘FR’ Power
#define SYSEN 0x48  //0b 0100 1000  EN 0: disable all blocks on-chip, all com/seg pin will be pulled to GND. 1: enable
#define LCDOFF 0x79  //0b 0111 1001  Turn off all LCD segments
#define LCDON 0x7A   //0b 0111 1010  Turn on all LCD segments

#define PIXONOFFDATA 0x78 //0b 0111 1000  All pixels are ON/OFF depending on the data in
#define NOBLINK 0x70      //0b 0111 0000  No blink
#define EV0 0x60          //0b 0110 0000 EV=0 Adjust resistor divider for LCD contrast setting.
#define NORMALMODE 0x20   //0b 0010 0000 80Hz Normal Mode, Line inverse, *0.5, Power Save Mode 1

#define SLAVE_OWN_ADDRESS 0x7C
#define MODE_CMD  0x01
#define MODE_DATA 0x00
#define ADR04_CMD 0x80
#define ADR56_CMD 0xe8

#define ADR0_SHIFT 0
#define ADR1_SHIFT 7

#define BAT1_SEG (1 << 4)
#define BAT2_SEG (1 << 0)
#define BAT3_SEG (1 << 1)
#define BAT4_SEG (1 << 5)

#define BAT14_POS 3
#define BAT23_POS 2

#define P1_SEG (1 << 7)
#define P1_POS 10
#define P2_SEG (1 << 7)
#define P2_POS 9
#define P3_SEG (1 << 7)
#define P3_POS 8
#define P4_SEG (1 << 7)
#define P4_POS 7
#define P5_SEG (1 << 7)
#define P5_POS 6

#define NUM1FGE_SEG 0x70 //0b01110000
#define NUM1FGE_POS 14
#define NUM1ABCD_SEG (0xf << 0) //0b00001111
#define NUM1ABCD_POS 13

#define MINUS_SEG (1 << 7)
#define MINUS_POS 13

#define MIN_RU_SEG (1 << 7)
#define MIN_RU_POS 12
#define MAX_RU_SEG (1 << 3)
#define MAX_RU_POS 14

#define MIN_EN_SEG (1 << 7)
#define MIN_EN_POS 11
#define MAX_EN_SEG (1 << 7)
#define MAX_EN_POS 14

#define BURST_RU_SEG (1 << 3)
#define BURST_RU_POS 15
#define BURST_EN_SEG (1 << 6)
#define BURST_EN_POS 15

#define LEAK_RU_SEG (1 << 5)
#define LEAK_RU_POS 15
#define LEAK_EN_SEG (1 << 2)
#define LEAK_EN_POS 15

#define REV_RU_SEG (1 << 1)
#define REV_RU_POS 15
#define REV_EN_SEG (1 << 0)
#define REV_EN_POS 15

#define FROST_SEG (1 << 7)
#define FROST_POS 15

#define Q_SEG (1 << 2)
#define Q_POS 14

#define VER_RU_SEG (1 << 0)
#define VER_RU_POS 14
#define VER_EN_SEG (1 << 1)
#define VER_EN_POS 14

#define POV_SEG (1 << 4)
#define POV_POS 15

#define SN_SEG (1 << 6)
#define SN_POS 0

#define WARN_SEG (1 << 7)
#define WARN_POS 0

#define MAGNET_SEG (1 << 3)
#define MAGNET_POS 0

#define LEFT_SEG (1 << 7)
#define LEFT_POS 1

#define RIGHT_SEG (1 << 7)
#define RIGHT_POS 2

#define NOWATER_SEG (1 << 3)
#define NOWATER_POS 1

#define CRC_SEG (1 << 5)
#define CRC_POS 0

#define DELTA_SEG (1 << 4)
#define DELTA_POS 0

#define T_SEG (1 << 1)
#define T_POS 0

#define T1_SEG (1 << 2)
#define T1_POS 0

#define T2_SEG (1 << 6)
#define T2_POS 1

#define NBFI_SEG (1 << 3)
#define NBFI_POS 2

#define NBIOT_SEG (1 << 7)
#define NBIOT_POS 3

#define SIG1_SEG (1 << 6)
#define SIG1_POS 2
#define SIG2_SEG (1 << 2)
#define SIG2_POS 2
#define SIG3_SEG (1 << 6)
#define SIG3_POS 3

#define SP_RU_SEG (1 << 7)
#define SP_RU_POS 4
#define SP_EN_SEG (1 << 4)
#define SP_EN_POS 3
#define RP_RU_SEG (1 << 6)
#define RP_RU_POS 4
#define RP_EN_SEG (1 << 5)
#define RP_EN_POS 3

#define DEGREE_SEG (1 << 5)
#define DEGREE_POS 1

#define GCAL_SEG (1 << 1)
#define GCAL_POS 3
#define GCAL_H_SEG (1 << 5)
#define GCAL_H_POS 4

#define GJ_SEG (1 << 4)
#define GJ_POS 1
#define GJ_H_SEG (1 << 0)
#define GJ_H_POS 3

#define KW_SEG (1 << 4)
#define KW_POS 5
#define MW_SEG (1 << 5)
#define MW_POS 5
#define W_SEG (1 << 4)
#define W_POS 4
#define WH_SEG (1 << 0)
#define WH_POS 4

#define GAL_SEG (1 << 6)
#define GAL_POS 5
#define GAL_PM_SEG (1 << 1)
#define GAL_PM_POS 4

#define M3_SEG (1 << 7)
#define M3_POS 5
#define M3_H_SEG (1 << 2)
#define M3_H_POS 4
#define M3_H_EN_SEG (1 << 3)
#define M3_H_EN_POS 4

#define FT3_SEG (1 << 1)
#define FT3_POS 1
#define FT3_PM_SEG (1 << 0)
#define FT3_PM_POS 1

#define MMBTU_SEG (1 << 2)
#define MMBTU_POS 1

#define GALLONS_SEG (1 << 4)
#define GALLONS_POS 2
#define US_SEG (1 << 5)
#define US_POS 2

#define ALL_CLEAR_SEG 0xff
#define ALL_CLEAR_POS 0

#define DOT_SEG 0x80

static const char ascii[] =
{
/*       0     1     2     3     4     5     6     7     8     9     a     b     c     d     e     f */
/*      ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   ' '   '-'   ' '   ' ' */
/*2*/   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
/*      '0'   '1'   '2'   '3'   '4'   '5'   '6'   '7'   '8'   '9'   ' '   ' '   ' '   ' '   ' '   ' ' */
/*3*/   0x7D, 0x60, 0x3e, 0x7a, 0x63, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*      ' '   'A'   'B'   'C'   'D'   'E'   'F'   'G'   'H'   'I'   'J'   'K'   'L'   'M'   'N'   'O' */
/*4*/   0x00, 0x77, 0x4f, 0x1d, 0x6e, 0x1f, 0x17, 0x5d, 0x47, 0x05, 0x68, 0x27, 0x0d, 0x54, 0x75, 0x4e,
/*      'P'   'Q'   'R'   'S'   'T'   'U'   'V'   'W'   'X'   'Y'   'Z'   ' '   ' '   ' '   ' '   '_' */
/*5*/   0x37, 0x73, 0x06, 0x59, 0x0f, 0x6d, 0x23, 0x29, 0x67, 0x6b, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x08,
/*      ' '   'A'   'B'   'C'   'D'   'E'   'F'   'G'   'H'   'I'   'J'   'K'   'L'   'M'   'N'   'O' */
/*6*/   0x00, 0x77, 0x4f, 0x1d, 0x6e, 0x1f, 0x17, 0x5d, 0x47, 0x05, 0x68, 0x27, 0x0d, 0x54, 0x75, 0x4e,
/*      'P'   'Q'   'R'   'S'   'T'   'U'   'V'   'W'   'X'   'Y'   'Z'   ' '   ' '   ' '   ' '   ' ' */
/*7*/   0x37, 0x73, 0x06, 0x59, 0x0f, 0x6d, 0x23, 0x29, 0x67, 0x6b, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define ASCII_SPACE_SYMBOL 0x00

#define LITTLE_ENDIAN

#if !defined(BIG_ENDIAN) && !defined(LITTLE_ENDIAN)
#error "Unable to determine endian. Set it manually"
#endif

union tDataSeq
{
    struct //__attribute__((packed))
    {
#if defined LITTLE_ENDIAN
        uint8_t padding : 7;
        uint64_t data1 : 64;
        uint64_t data0 : 64;
        uint8_t addr : 6;
        uint8_t type : 3;
#elif defined BIG_ENDIAN
        uint8_t type : 3;
        uint8_t addr : 6;
        uint16_t data0 : 16;
        uint16_t data1 : 16;
        uint16_t data2 : 16;
        uint8_t padding : 7;
#endif
    };
    uint8_t arr[18];
};

CN91C4S96_HAL_st *CN91C4S96_hal = 0;

// the most low-level function. Sends array of bytes into display
void wrBytes(uint8_t *ptr, uint8_t size);
// write Buffer to the display
void wrBuffer();
// write command sequence to display
void wrCmd(uint8_t cmd);
// set decimal separator. Used when print float numbers
void decimalSeparator(uint8_t dpPosition);
// set two dots for date
void dateSeparator(uint8_t dpPosition, uint8_t dpPosition2);
// check if number below minimum number with segment minus, show minus and abs num
void showMinus(int32_t *num);
// takes the Buffer and puts it straight into the driver
void update();
// remove battery symbol from display Buffer
void batteryBufferClear();
// remove dot symbol from display Buffer
void dotsBufferClear();
// remove all symbols from display Buffer except battery and dots
void lettersBufferClear();
//Clear all segments
void AllClear();
// coverts Buffer symbols to format, which can be displayed by LCD
void BufferToAscii(const char *in, uint8_t *out);

void CN91C4S96Init(CN91C4S96_HAL_st *hal_ptr)
{
    assert_param(hal_ptr->InitI2C != NULL);
    assert_param(hal_ptr->WriteI2C != NULL);
    assert_param(hal_ptr->WaitI2C != NULL);
    CN91C4S96_hal = hal_ptr;

    CN91C4S96_hal->InitI2C(); //

    wrCmd(ULP | SYSEN);
//        wrCmd(PIXONOFFDATA);
//        wrCmd(NOBLINK);
//        wrCmd(EV0);
//        wrCmd(NORMALMODE);

}

void CN91C4S96displayOn()
{
    wrCmd(LCDON);
}

void CN91C4S96displayOff()
{
    wrCmd(LCDOFF);
}

void CN91C4S96displayData()
{
    wrCmd(PIXONOFFDATA);
}

void *reverseBytes(void *inp, size_t len)
{
    unsigned int i;
    unsigned char *in = (unsigned char *)inp, tmp;

    for (i = 0; i < len / 2; i++)
    {
        tmp = *(in + i);
        *(in + i) = *(in + len - i - 1);
        *(in + len - i - 1) = tmp;
    }

    return inp;
}

void wrBytes(uint8_t *ptr, uint8_t size)
{
    // TODO: check wrong size
    assert_param(CN91C4S96_hal->WriteI2C != NULL);
    CN91C4S96_hal->WriteI2C(SLAVE_OWN_ADDRESS, ptr, size);
}

void wrBuffer()
{
    BufferSend[0] = ADR56_CMD;
    BufferSend[1] = MODE_DATA;
    if(memcmp(BufferSendOld, BufferSend, DISPLAY_BUFFER_SIZE) != 0)
    {
        memcpy(BufferSendOld, BufferSend, sizeof(BufferSend));
        wrBytes(BufferSend, sizeof(BufferSend));
    }

}

void wrCmd(uint8_t cmd)
{
    assert_param(CN91C4S96_hal->WaitI2C != NULL);
    union
    {
        struct __attribute__((packed))
        {
#if defined LITTLE_ENDIAN
            uint8_t data : 7;
            uint8_t type : 1;
#elif defined BIG_ENDIAN
            uint8_t type : 1;
            uint8_t data : 7;
#endif
        };
        uint8_t arr[1];
    } CommandSeq;
    CommandSeq.type = MODE_CMD;
    CommandSeq.data = cmd;

    wrBytes(CommandSeq.arr, sizeof(CommandSeq));
    CN91C4S96_hal->WaitI2C();
}

void CN91C4S96batteryLevel(uint8_t percents)
{
    batteryBufferClear();
    SET_BIT(Buffer[BAT14_POS], BAT4_SEG);
    if (percents > 75)
    {
        SET_BIT(Buffer[BAT14_POS], BAT1_SEG);
    }
    if (percents > 50)
    {
        SET_BIT(Buffer[BAT23_POS], BAT2_SEG);
    }
    if (percents > 25)
    {
        SET_BIT(Buffer[BAT23_POS], BAT3_SEG);
    }
}

void batteryBufferClear()
{
    CLEAR_BIT(Buffer[BAT14_POS], BAT1_SEG | BAT4_SEG);
    CLEAR_BIT(Buffer[BAT23_POS], BAT2_SEG | BAT3_SEG);
}

void dotsBufferClear()
{
    for (size_t i = 0; i < PRECISION_MAX_POSITIVE; i++)
    {
        CLEAR_BIT(Buffer[P5_POS + i], P5_SEG);
    }
}

void lettersBufferClear()
{
    for (size_t i = 0; i < DISPLAY_SIZE; i++)
    {
        CLEAR_BIT(Buffer[NUM1FGE_POS - i], NUM1FGE_SEG);
        CLEAR_BIT(Buffer[NUM1ABCD_POS - i], NUM1ABCD_SEG);
    }
}

void AllClear()
{
    CLEAR_BIT(Buffer[ALL_CLEAR_POS], ALL_CLEAR_SEG);
    for (size_t i = 0; i < DATA_SIZE; i++)
    {
        Buffer[i + SYS_SIZE] = 0;
    }
}
void clear()
{
    AllClear();
}

void BufferToAscii(const char *in, uint8_t *out)
{
    for (size_t i = 0; i < MIN(DISPLAY_SIZE, strlen(in)); i++)
    {
        char c = in[i];
        // Handle situation when char is out of displayable ascii table part.
        // Show space instead
        if ((c < ' ') || (c - ' ' >= (int)sizeof(ascii)))
        {
            CLEAR_BIT(out[NUM1FGE_POS - i], NUM1FGE_SEG);
            CLEAR_BIT(out[NUM1ABCD_POS - i], NUM1ABCD_SEG);
        }
        else
        {
            SET_BIT(out[NUM1FGE_POS - i], NUM1FGE_SEG & (ascii[c - ' '] << 4));   // shift 4 for changing data format from library to our display
            SET_BIT(out[NUM1ABCD_POS - i], NUM1ABCD_SEG & (ascii[c - ' '] >> 4)); // shift 4 for changing data format from library to our display
            SET_BIT(out[NUM1ABCD_POS - i], NUM1ABCD_SEG & ascii[c - ' '] & 0x08); // mask for changing D segment from library to our display
        }
    }
}

void CN91C4S96printStr(const char *str)
{
    dotsBufferClear();
    lettersBufferClear();
    BufferToAscii(str, Buffer);
}

void CN91C4S96printNum(int32_t num, int32_t precision)
{
    if (num > MAX_NUM)
        num = MAX_NUM;
    if (num < MIN_NUM)
        num = MIN_NUM;

    dotsBufferClear();
    lettersBufferClear();

    showMinus(&num);

    char str[DISPLAY_SIZE + 1] = {0};
    char strNum[DISPLAY_SIZE + 1] = {0};
    int8_t numberSymbols = (int32_t)log10(num) + 1;
    for (int8_t i = 0; i < DISPLAY_SIZE - numberSymbols; i++)
    {
        if (i < DISPLAY_SIZE - precision - 1)
        {
            str[i] = ' ';
        }
        else
        {
            str[i] = '0';
        }
    }
#if __STDC_WANT_LIB_EXT1__ == 1
    snprintf_s(str, sizeof(str), "%9li", num);
#else
    snprintf(strNum, sizeof(strNum), "%9li", num);
#endif
    for (int32_t i = DISPLAY_SIZE - numberSymbols; i < DISPLAY_SIZE; i++)
    {
        str[i] = strNum[i];
    }
    BufferToAscii(str, Buffer);
}

void CN91C4S96printFloat(float num, uint8_t precision)
{
    if (num >= 0 && precision > PRECISION_MAX_POSITIVE)
        precision = PRECISION_MAX_POSITIVE;
    else if (num < 0 && precision > PRECISION_MAX_NEGATIVE)
        precision = PRECISION_MAX_NEGATIVE;

    if (num < (float)MIN_NUM / 10)
    {
        num = num * (-1);
        SET_BIT(Buffer[MINUS_POS], MINUS_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[MINUS_POS], MINUS_SEG);
    }

    int32_t integerated = (int32_t)(num * pow(10, precision));

    if (integerated > MAX_NUM)
        integerated = MAX_NUM;
    if (integerated < MIN_NUM)
        integerated = MIN_NUM;

    CN91C4S96printNum(integerated, precision);
    decimalSeparator(precision);
}

// TODO: make multiplier more strict.
void CN91C4S96printFixed(int32_t multiplied_float, uint32_t multiplier)
{
    uint8_t precision = 0;

    if (multiplier == 100000)
        precision = 5;
    else if (multiplier == 10000)
        precision = 4;
    else if (multiplier == 1000)
        precision = 3;
    else if (multiplier == 100)
        precision = 2;
    else if (multiplier == 10)
        precision = 1;
    else
        precision = 0;

    if (multiplied_float > MAX_NUM)
        multiplied_float = MAX_NUM;
    if (multiplied_float < MIN_NUM)
        multiplied_float = MIN_NUM;

    CN91C4S96printNum((int32_t)multiplied_float, precision);
    decimalSeparator(precision);
}

void CN91C4S96printDate(int32_t day, int32_t mon, int32_t year)
{
    char str[DISPLAY_SIZE + 1] = {"   "};
    char strDate[DISPLAY_SIZE + 1] = {0};
#if __STDC_WANT_LIB_EXT1__ == 1
    snprintf_s(strDate, sizeof(strDate), "%02i%02i%02i", day, mon, year);
    strcat_s(str, sizeof(str), strDate);
#else
    snprintf(strDate, sizeof(strDate), "%02i%02i%02i", day, mon, year);
    strcat(str, strDate);
#endif


    BufferToAscii(str, Buffer);
    dateSeparator(2, 4);
}

void decimalSeparator(uint8_t dpPosition)
{
    dotsBufferClear();

    if (dpPosition < PRECISION_MIN || dpPosition > PRECISION_MAX_POSITIVE)
        // selected dot position not supported by display hardware
        return;

    SET_BIT(Buffer[P1_POS - PRECISION_MAX_POSITIVE + dpPosition], P1_SEG);
}

void dateSeparator(uint8_t dpPosition, uint8_t dpPosition2)
{
    dotsBufferClear();

    if (dpPosition < PRECISION_MIN || dpPosition > PRECISION_MAX_POSITIVE)
        // selected dot position not supported by display hardware
        return;
    if (dpPosition2 < PRECISION_MIN || dpPosition2 > PRECISION_MAX_POSITIVE)
      // selected dot position not supported by display hardware
      return;

    SET_BIT(Buffer[P1_POS - PRECISION_MAX_POSITIVE + dpPosition], P1_SEG);
    SET_BIT(Buffer[P1_POS - PRECISION_MAX_POSITIVE + dpPosition2], P1_SEG);
}

void showMinus(int32_t *num)
{
    if (*num < MIN_MINUS_NUM)
    {
        SET_BIT(Buffer[MINUS_POS], MINUS_SEG);
        *num *= -1;
    }
    else
    {
        CLEAR_BIT(Buffer[MINUS_POS], MINUS_SEG);
    }
}

void CN91C4S96DispMinMax(bool enable, bool mode, bool min)
{
    if (enable)
    {
        if (mode)
        {
            LCD_TOGGLE(min, MIN_RU_POS, MIN_RU_SEG, MAX_RU_POS, MAX_RU_SEG);
        }
        else
        {
            LCD_TOGGLE(min, MIN_EN_POS, MIN_EN_SEG, MAX_EN_POS, MAX_EN_SEG);
        }
    }
    else
    {
        CLEAR_BIT(Buffer[MIN_RU_POS], MIN_RU_SEG);
        CLEAR_BIT(Buffer[MAX_RU_POS], MAX_RU_SEG);
        CLEAR_BIT(Buffer[MIN_EN_POS], MIN_EN_SEG);
        CLEAR_BIT(Buffer[MAX_EN_POS], MAX_EN_SEG);
    }
}

void CN91C4S96DispBurst(bool enable, bool mode)
{
    if (enable)
    {
        LCD_TOGGLE(mode, BURST_RU_POS, BURST_RU_SEG, BURST_EN_POS, BURST_EN_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[BURST_RU_POS], BURST_RU_SEG);
        CLEAR_BIT(Buffer[BURST_EN_POS], BURST_EN_SEG);
    }
}

void CN91C4S96DispLeak(bool enable, bool mode)
{
    if (enable)
    {
        LCD_TOGGLE(mode, LEAK_RU_POS, LEAK_RU_SEG, LEAK_EN_POS, LEAK_EN_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[LEAK_RU_POS], LEAK_RU_SEG);
        CLEAR_BIT(Buffer[LEAK_EN_POS], LEAK_EN_SEG);
    }
}

void CN91C4S96DispRev(bool enable, bool mode)
{
    if (enable)
    {
        LCD_TOGGLE(mode, REV_RU_POS, REV_RU_SEG, REV_EN_POS, REV_EN_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[REV_RU_POS], REV_RU_SEG);
        CLEAR_BIT(Buffer[REV_EN_POS], REV_EN_SEG);
    }
}

void CN91C4S96DispFrost(bool enable)
{
    LCD_SWITCH(enable, FROST_POS, FROST_SEG);
}

void CN91C4S96DispQ(bool enable)
{
    LCD_SWITCH(enable, Q_POS, Q_SEG);
}

void CN91C4S96DispVer(bool enable, bool mode)
{
    if (enable)
    {
        LCD_TOGGLE(mode, VER_RU_POS, VER_RU_SEG, VER_EN_POS, VER_EN_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[VER_RU_POS], VER_RU_SEG);
        CLEAR_BIT(Buffer[VER_EN_POS], VER_EN_SEG);
    }
}

void CN91C4S96DispSN(bool enable)
{
    if (enable)
    {
        SET_BIT(Buffer[SN_POS], SN_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[SN_POS], SN_SEG);
    }
}

void CN91C4S96DispWarn(bool enable)
{
    LCD_SWITCH(enable, WARN_POS, WARN_SEG);
}

void CN91C4S96DispMagn(bool enable)
{
    LCD_SWITCH(enable, MAGNET_POS, MAGNET_SEG);
}

void CN91C4S96DispLeft(bool enable)
{
    LCD_SWITCH(enable, LEFT_POS, LEFT_SEG);
}

void CN91C4S96DispRight(bool enable)
{
    LCD_SWITCH(enable, RIGHT_POS, RIGHT_SEG);
}

void CN91C4S96DispNoWater(bool enable)
{
    LCD_SWITCH(enable, NOWATER_POS, NOWATER_SEG);
}

void CN91C4S96DispSP(bool enable, bool mode)
{
    if (enable)
    {
        if (mode)
        {
            SET_BIT(Buffer[SP_RU_POS], SP_RU_SEG);
        }
        else
        {
            SET_BIT(Buffer[SP_EN_POS], SP_EN_SEG);
        }
    }
    else
    {
        CLEAR_BIT(Buffer[SP_RU_POS], SP_RU_SEG);
        CLEAR_BIT(Buffer[SP_EN_POS], SP_EN_SEG);
    }
}

void CN91C4S96DispRP(bool enable, bool mode)
{
    if (enable)
    {
        if (mode)
        {
            SET_BIT(Buffer[RP_RU_POS], RP_RU_SEG);
        }
        else
        {
            SET_BIT(Buffer[RP_EN_POS], RP_EN_SEG);
        }
    }
    else
    {
        CLEAR_BIT(Buffer[RP_RU_POS], RP_RU_SEG);
        CLEAR_BIT(Buffer[RP_EN_POS], RP_EN_SEG);
    }
}

void CN91C4S96DispCRC(bool enable)
{
    LCD_SWITCH(enable, CRC_POS, CRC_SEG);
}

void CN91C4S96DispDelta(bool enable)
{
    LCD_SWITCH(enable, DELTA_POS, DELTA_SEG);
}

void CN91C4S96DispT(bool enable)
{
    LCD_SWITCH(enable, T_POS, T_SEG);
}

void CN91C4S96Disp1(bool enable)
{
    LCD_SWITCH(enable, T1_POS, T1_SEG);
}

void CN91C4S96DispT2(bool enable)
{
    LCD_SWITCH(enable, T2_POS, T2_SEG);
}

void CN91C4S96DispNBFi(bool enable)
{
    LCD_SWITCH(enable, NBFI_POS, NBFI_SEG);
}

void CN91C4S96DispNBIoT(bool enable)
{
    LCD_SWITCH(enable, NBIOT_POS, NBIOT_SEG);
}

void CN91C4S96SignalLevel(uint8_t percents)
{
    CLEAR_BIT(Buffer[SIG1_POS], SIG1_SEG);
    CLEAR_BIT(Buffer[SIG2_POS], SIG2_SEG);
    CLEAR_BIT(Buffer[SIG3_POS], SIG3_SEG);
    if (percents > 60)
    {
        SET_BIT(Buffer[SIG3_POS], SIG3_SEG);
    }
    if (percents > 30)
    {
        SET_BIT(Buffer[SIG2_POS], SIG2_SEG);
    }
    if (percents > 0)
    {
        SET_BIT(Buffer[SIG1_POS], SIG1_SEG);
    }
}

void CN91C4S96DispDegreePoint(bool enable)
{
    LCD_SWITCH(enable, DEGREE_POS, DEGREE_SEG);
}

void CN91C4S96DispEnergyJ(bool enable, bool mode, bool perH)
{
    if (enable)
    {
        if (mode)
        {
            SET_BIT(Buffer[GCAL_POS], GCAL_SEG);
            LCD_SWITCH(perH, GCAL_H_POS, GCAL_H_SEG);
        }
        else
        {
            SET_BIT(Buffer[GJ_POS], GJ_SEG);
            LCD_SWITCH(perH, GJ_H_POS, GJ_H_SEG);
        }
    }
    else
    {
        CLEAR_BIT(Buffer[GJ_POS], GJ_SEG);
        CLEAR_BIT(Buffer[GJ_H_POS], GJ_H_SEG);
        CLEAR_BIT(Buffer[GCAL_POS], GCAL_SEG);
        CLEAR_BIT(Buffer[GCAL_H_POS], GCAL_H_SEG);
    }
}

void CN91C4S96DispEnergyW(bool enable, bool M, bool perH)
{
    if (enable)
    {
        SET_BIT(Buffer[W_POS], W_SEG);
        LCD_TOGGLE(M, MW_POS, MW_SEG, KW_POS, KW_SEG);
        LCD_SWITCH(perH, WH_POS, WH_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[W_POS], W_SEG);
        CLEAR_BIT(Buffer[KW_POS], KW_SEG);
        CLEAR_BIT(Buffer[MW_POS], MW_SEG);
        CLEAR_BIT(Buffer[WH_POS], WH_SEG);
    }
}

void CN91C4S96DispFlowM3(bool enable, bool mode, bool perH)
{
    if (enable)
    {
        SET_BIT(Buffer[M3_POS], M3_SEG);
        if (perH)
        {
            SET_BIT(Buffer[M3_H_POS], M3_H_SEG);
            LCD_SWITCH(!mode, M3_H_EN_POS, M3_H_EN_SEG);
        }
        else
        {
            CLEAR_BIT(Buffer[M3_H_POS], M3_H_SEG);
            CLEAR_BIT(Buffer[M3_H_EN_POS], M3_H_EN_SEG);
        }
    }
    else
    {
        CLEAR_BIT(Buffer[M3_POS], M3_SEG);
        CLEAR_BIT(Buffer[M3_H_POS], M3_H_SEG);
        CLEAR_BIT(Buffer[M3_H_EN_POS], M3_H_EN_SEG);
    }
}

void CN91C4S96DispFlowGAL(bool enable, bool perH)
{
    if (enable)
    {
        SET_BIT(Buffer[GAL_POS], GAL_SEG);
        LCD_SWITCH(perH, GAL_PM_POS, GAL_PM_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[GAL_POS], GAL_SEG);
        CLEAR_BIT(Buffer[GAL_PM_POS], GAL_PM_SEG);
    }
}

void CN91C4S96DispFlowFT(bool enable, bool perH)
{
    if (enable)
    {
        SET_BIT(Buffer[FT3_POS], FT3_SEG);
        LCD_SWITCH(perH, FT3_PM_POS, FT3_PM_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[FT3_POS], FT3_SEG);
        CLEAR_BIT(Buffer[FT3_PM_POS], FT3_PM_SEG);
    }
}

void CN91C4S96DispMMBTU(bool enable)
{
    LCD_SWITCH(enable, MMBTU_POS, MMBTU_SEG);
}

void CN91C4S96DispGal(bool enable, bool mode)
{
    if (enable)
    {
        SET_BIT(Buffer[GALLONS_POS], GALLONS_SEG);
        LCD_SWITCH(mode, US_POS, US_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[GALLONS_POS], GALLONS_SEG);
        CLEAR_BIT(Buffer[US_POS], US_SEG);
    }
}

void CN91C4S96DispPOV(bool enable)
{
    if (enable)
    {
        SET_BIT(Buffer[POV_POS], POV_SEG);
        SET_BIT(Buffer[VER_RU_POS], VER_RU_SEG);
    }
    else
    {
        CLEAR_BIT(Buffer[POV_POS], POV_SEG);
        CLEAR_BIT(Buffer[VER_RU_POS], VER_RU_SEG);
    }
}

void CN91C4S96DispWrite(void)
{
    wrBuffer();
}