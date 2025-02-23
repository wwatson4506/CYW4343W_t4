/**
 * Copyright (c) 2011-2021 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


 //Broadcom full MAC driver source used for reference:
 //https://github.com/torvalds/linux/releases/tag/v5.6-rc4

#include <Arduino.h>
#include "SdioTeensy4.h"
#include "SdCardInfo.h"
#include "SdioCard.h"
#include "SdioRegs.h"
#include "misc_defs.h"
#include "ioctl.h"

//Remove this define to use built-in internal LPO in 4343W
//#define USE_EXTERNAL_LPO

///////////////
//Firmware file
///////////////

#include "../firmware/brcmfmac43430-sdio.c"                  // Zero:  Oct 23 2017 03:55:53 version 7.45.98.38
//#include "../firmware/w4343WA1_7_45_98_50_combined.h"      // CYW43: Apr 30 2018 04:14:19 version 7.45.98.50
//#include "../firmware/w4343WA1_7_45_98_102_combined.h"     // CYW43: Jun 18 2020 08:48:22 version 7.45.98.102 
//#include "../firmware/cyfmac43430_fmac_7_45_98_125-sdio.c" // fmac:  Aug 16 2022 03:05:14 version 7.45.98.125

////////////
//NVRAM file
/////////////
#include "../firmware/wifi_nvram_4343W_zero.h"
//#include "../firmware/wifi_nvram_1dx.h"



//==============================================================================
//------------------------------------------------------------------------------
//==============================================================================
W4343WCard *W4343WCard::s_pSdioCards[2] = {nullptr, nullptr};
volatile bool W4343WCard::dataISRReceived = false;
volatile bool W4343WCard::fUseSDIO2 = false;

wl_country_t country_struct = {.country_abbrev=COUNTRY, .rev=COUNTRY_REV, .ccode=COUNTRY};
#define CHECK(f, a, ...) {if (!f(a, __VA_ARGS__)) \
                          Serial.printf("Error: %s(%s ...)\n", #f, #a);}

#define DBG_TRACE Serial.print("TRACE."); Serial.println(__LINE__); delay(200);
#define USE_DEBUG_MODE 1
#if USE_DEBUG_MODE
#define DBG_IRQSTAT() if (m_psdhc->INT_STATUS) {Serial.print(__LINE__);\
        Serial.print(" IRQSTAT "); Serial.print(SER_RED); Serial.println(m_psdhc->INT_STATUS, HEX); Serial.print(SER_RESET);}
void W4343WCard::printRegs(uint32_t line) {
  uint32_t blkattr = m_psdhc->BLK_ATT;
  uint32_t xfertyp = m_psdhc->CMD_XFR_TYP;
  uint32_t prsstat = m_psdhc->PRES_STATE;
  uint32_t proctl = m_psdhc->PROT_CTRL;
  uint32_t irqstat = m_psdhc->INT_STATUS;
  Serial.print("\nLINE: ");
  Serial.println(line);
  Serial.print("BLKATTR ");
  Serial.println(blkattr, HEX);
  Serial.print("XFERTYP ");
  Serial.print(xfertyp, HEX);
  Serial.print(" CMD");
  Serial.print(xfertyp >> 24);
  Serial.print(" TYP");
  Serial.print((xfertyp >> 2) & 3);
  if (xfertyp & SDHC_XFERTYP_DPSEL) {Serial.print(" DPSEL");}
  Serial.println();
  Serial.print("PRSSTAT ");
  Serial.print(prsstat, HEX);
  if (prsstat & SDHC_PRSSTAT_BREN) {Serial.print(" BREN");}
  if (prsstat & SDHC_PRSSTAT_BWEN) {Serial.print(" BWEN");}
  if (prsstat & SDHC_PRSSTAT_RTA) {Serial.print(" RTA");}
  if (prsstat & SDHC_PRSSTAT_WTA) {Serial.print(" WTA");}
  if (prsstat & SDHC_PRSSTAT_SDOFF) {Serial.print(" SDOFF");}
  if (prsstat & SDHC_PRSSTAT_PEROFF) {Serial.print(" PEROFF");}
  if (prsstat & SDHC_PRSSTAT_HCKOFF) {Serial.print(" HCKOFF");}
  if (prsstat & SDHC_PRSSTAT_IPGOFF) {Serial.print(" IPGOFF");}
  if (prsstat & SDHC_PRSSTAT_SDSTB) {Serial.print(" SDSTB");}
  if (prsstat & SDHC_PRSSTAT_DLA) {Serial.print(" DLA");}
  if (prsstat & SDHC_PRSSTAT_CDIHB) {Serial.print(" CDIHB");}
  if (prsstat & SDHC_PRSSTAT_CIHB) {Serial.print(" CIHB");}
  Serial.println();
  Serial.print("PROCTL ");
  Serial.print(proctl, HEX);
  if (proctl & SDHC_PROCTL_SABGREQ) Serial.print(" SABGREQ");
  Serial.print(" EMODE");
  Serial.print((proctl >>4) & 3);
  Serial.print(" DWT");
  Serial.print((proctl >>1) & 3);
  Serial.println();
  Serial.print("IRQSTAT ");
  Serial.print(irqstat, HEX);
  if (irqstat & SDHC_IRQSTAT_BGE) {Serial.print(" BGE");}
  if (irqstat & SDHC_IRQSTAT_TC) {Serial.print(" TC");}
  if (irqstat & SDHC_IRQSTAT_CC) {Serial.print(" CC");}
  Serial.print("\nm_irqstat ");
  Serial.println(m_irqstat, HEX);
}
#else  // USE_DEBUG_MODE
#define DBG_IRQSTAT()
#endif  // USE_DEBUG_MODE
//==============================================================================
// Error function and macro.
#define sdError(code) setSdErrorCode(code, __LINE__)
inline bool W4343WCard::setSdErrorCode(uint8_t code, uint32_t line) {
  m_errorCode = code;
  m_errorLine = line;
#if USE_DEBUG_MODE
  printRegs(line);
#endif  // USE_DEBUG_MODE
  return false;
}
//==============================================================================
// ISR
void W4343WCard::sdISR() {
  USDHC1_INT_SIGNAL_EN = 0;
  s_pSdioCards[0]->m_irqstat = USDHC1_INT_STATUS;
  USDHC1_INT_STATUS = s_pSdioCards[0]->m_irqstat;
  USDHC1_MIX_CTRL &= ~(SDHC_MIX_CTRL_AC23EN | SDHC_MIX_CTRL_DMAEN);
  s_pSdioCards[0]->m_dmaBusy = false;
}

void W4343WCard::sdISR2() {
  USDHC2_INT_SIGNAL_EN = 0;
  s_pSdioCards[1]->m_irqstat = USDHC2_INT_STATUS;
  USDHC2_INT_STATUS = s_pSdioCards[1]->m_irqstat;
  USDHC2_MIX_CTRL &= ~(SDHC_MIX_CTRL_AC23EN | SDHC_MIX_CTRL_DMAEN);
  s_pSdioCards[1]->m_dmaBusy = false;
}


//==============================================================================
// GPIO and clock functions.

//------------------------------------------------------------------------------
void W4343WCard::gpioMux(uint8_t mode) {
  if (fUseSDIO2 == false) {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = mode;  // DAT3
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04 = mode;  // DAT2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00 = mode;  // CMD
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01 = mode;  // CLK
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02 = mode;  // DAT0
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = mode;  // DAT1
  } else {
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_07 = mode; // USDHC2_DATA3
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_06 = mode; // USDHC2_DATA2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_08 = mode; // USDHC2_CMD
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = mode; // USDHC2_CLK
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = mode; // USDHC2_DATA0
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = mode; // USDHC2_DATA1
  }
}

// Switches DAT1 to SDIO mode
void W4343WCard::makeSDIO_DAT1()
{
  fUseSDIO2 == false ? IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x00 : IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0x06;
}

//Switches DAT1 to GPIO mode to enable interrupt when data ready
void W4343WCard::makeGPIO_DAT1()
{
  fUseSDIO2 == false ? IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03 = 0x05 : IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 0x05;
}

//------------------------------------------------------------------------------
// add speed strength args?
void W4343WCard::enableSDIO(bool enable) {
  const uint32_t CLOCK_MASK = IOMUXC_SW_PAD_CTL_PAD_PKE |
                              IOMUXC_SW_PAD_CTL_PAD_DSE(7) |
                              IOMUXC_SW_PAD_CTL_PAD_SPEED(2);

  const uint32_t DATA_MASK = CLOCK_MASK | IOMUXC_SW_PAD_CTL_PAD_PUE |
                             IOMUXC_SW_PAD_CTL_PAD_PUS(1);
  if (enable == true) {
    gpioMux(fUseSDIO2? 6 : 0);  // Default function. SDIO1 ALT0, SDIO2 is on ALT6
    if (fUseSDIO2 == false) {
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_04 = DATA_MASK;   // DAT2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_05 = DATA_MASK;   // DAT3
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_00 = DATA_MASK;   // CMD
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_01 = CLOCK_MASK;  // CLK
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_02 = DATA_MASK;   // DAT0
      IOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_03 = DATA_MASK;   // DAT1
    } else {      
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_07 = DATA_MASK;  // USDHC2_DATA3
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_06 = DATA_MASK;  // USDHC2_DATA2
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_08 = DATA_MASK;  // USDHC2_CMD
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 = CLOCK_MASK; // USDHC2_CLK
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = DATA_MASK;  // USDHC2_DATA0
      IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = DATA_MASK;  // USDHC2_DATA1

      // We need to set select input bits...
      IOMUXC_USDHC2_CLK_SELECT_INPUT = 0x1;
      IOMUXC_USDHC2_CMD_SELECT_INPUT = 0x01;
      IOMUXC_USDHC2_DATA0_SELECT_INPUT = 0x01;
      IOMUXC_USDHC2_DATA1_SELECT_INPUT = 0x01;
      IOMUXC_USDHC2_DATA2_SELECT_INPUT = 0x01;
      IOMUXC_USDHC2_DATA3_SELECT_INPUT = 0x01;
    }
  } else {
    gpioMux(5); //GPIO function
  }
}
//------------------------------------------------------------------------------
void W4343WCard::initClock() {
  /* set PDF_528 PLL2PFD0 */
  CCM_ANALOG_PFD_528 |= (1 << 7);
  CCM_ANALOG_PFD_528 &= ~(0x3F << 0);
  CCM_ANALOG_PFD_528 |= ((24) & 0x3F << 0);  // 12 - 35
  CCM_ANALOG_PFD_528 &= ~(1 << 7);

  /* Enable USDHC clock. */
  if (fUseSDIO2 == false) {
    CCM_CCGR6 |= CCM_CCGR6_USDHC1(CCM_CCGR_ON);
    CCM_CSCDR1 &= ~(CCM_CSCDR1_USDHC1_PODF(7));
    CCM_CSCMR1 |= CCM_CSCMR1_USDHC1_CLK_SEL;          // PLL2PFD0
  //  CCM_CSCDR1 |= CCM_CSCDR1_USDHC1_CLK_PODF((7)); / &0x7  WHG
    CCM_CSCDR1 |= CCM_CSCDR1_USDHC1_CLK_PODF((1));
  } else {
    CCM_CCGR6 |= CCM_CCGR6_USDHC2(CCM_CCGR_ON);
    CCM_CSCDR1 &= ~(CCM_CSCDR1_USDHC2_PODF(7));
    CCM_CSCMR1 |= CCM_CSCMR1_USDHC2_CLK_SEL;          // PLL2PFD0
  //  CCM_CSCDR1 |= CCM_CSCDR1_USDHC2_CLK_PODF((7)); / &0x7  WHG
    CCM_CSCDR1 |= CCM_CSCDR1_USDHC2_PODF(1);
  }
}
//------------------------------------------------------------------------------
uint32_t W4343WCard::baseClock() {
  uint32_t divider = ((CCM_CSCDR1 >> 11) & 0x7) + 1;
  return (528000000U * 3)/((CCM_ANALOG_PFD_528 & 0x3F)/6)/divider;
}
//==============================================================================
// Static functions

////////////////////
// IRW New functions
////////////////////
// Union to handle 8/16/32 bit conversions
typedef union
{
  int32_t  int32;
  uint32_t uint32;
  uint32_t uint24:24;
  uint16_t uint16;
  uint8_t  uint8;
  uint8_t  bytes[4];
} u32Data;


bool W4343WCard::cardCMD52_read(uint32_t functionNumber, uint32_t registerAddress, uint8_t * response, bool logOutput) {
  makeSDIO_DAT1();
  bool res = cardCMD52(functionNumber, registerAddress, 0, SD_RD, 0, response, logOutput);
  makeGPIO_DAT1();
  return res;
}

bool W4343WCard::cardCMD52_write(uint32_t functionNumber, uint32_t registerAddress, uint8_t data, bool logOutput) {
  makeSDIO_DAT1();
  bool res = cardCMD52(functionNumber, registerAddress, data, SD_WR, 0, NULL, logOutput);
  makeGPIO_DAT1();
  return res;
}

bool W4343WCard::cardCMD52(uint32_t functionNumber, uint32_t registerAddress, uint8_t data, bool write, bool readAfterWriteFlag, uint8_t * buffer, bool logOutput) {

  uint32_t arg = (functionNumber << 28);          // Function number in bits 28–30
  arg |= ((registerAddress & 0x1FFFF) << 9);      // Register address, 17 bits in 9-26
  
  if (write == true) {
    arg |= (1 << 31) | ((uint32_t)readAfterWriteFlag << 27); // Set write bit 31 if writing, raw flag bit 27
    arg |= (uint8_t)data;                                    // Data to write in bits 0–8
  } 

  // Issue the command
  if (!cardCommand(CMD52_XFERTYP, arg)) {
    Serial.println("CMD52 failed");
    return false;
  }

  if (logOutput) Serial.printf("CMD52 %s, 0x%08X, response: 0x%02X\n", write == true ? "write" : "read", registerAddress, m_psdhc->CMD_RSP0 & 0xFF);

  // If reading, extract the response byte
  if ((write == false) && buffer) {
    *buffer = m_psdhc->CMD_RSP0 & 0xFF;  //Read the response register, the read byte is in bits 0–7
  }

  return true;
}

void W4343WCard::setBlockCountSize(bool blockMode, uint32_t functionNumber, uint32_t size)
{
  // Set up block count and block size
  if (blockMode == true) {
    switch (functionNumber) {
      //TODO What is the correct BLKSIZE for SD_FUNC_BUS?
      case SD_FUNC_BUS: m_psdhc->BLK_ATT = SDHC_BLKATTR_BLKCNT(size) | SDHC_BLKATTR_BLKSIZE(64); 
                        break;
      case SD_FUNC_BAK: m_psdhc->BLK_ATT = SDHC_BLKATTR_BLKCNT(size) | SDHC_BLKATTR_BLKSIZE(SD_BAK_BLK_BYTES);
                        break;
      case SD_FUNC_RAD: m_psdhc->BLK_ATT = SDHC_BLKATTR_BLKCNT(size) | SDHC_BLKATTR_BLKSIZE(SD_RAD_BLK_BYTES);
                        break;
    }
  } else {
    m_psdhc->BLK_ATT = SDHC_BLKATTR_BLKCNT(1) | SDHC_BLKATTR_BLKSIZE(size);
  }
}

bool W4343WCard::cardCMD53_read(uint32_t functionNumber, uint32_t registerAddress, uint8_t * buffer, uint32_t size, bool logOutput)
{
  makeSDIO_DAT1();
  bool res = cardCMD53(functionNumber, registerAddress, buffer, size, false, logOutput);
  makeGPIO_DAT1();
  return res;
}

bool W4343WCard::cardCMD53_write(uint32_t functionNumber, uint32_t registerAddress, uint8_t * buffer, uint32_t size, bool logOutput) 
{
  makeSDIO_DAT1();
  bool res = cardCMD53(functionNumber, registerAddress, buffer, size, true, logOutput);
  makeGPIO_DAT1();
  return res;
}

bool W4343WCard::cardCMD53(uint32_t functionNumber, uint32_t registerAddress, uint8_t * buffer, uint32_t size, bool write, bool logOutput) {
  bool return_value = false;
  bool blockMode = false;
  uint8_t opCode = 1;
    // CMD53 argument format:
    // [31]    - Read/Write flag 
    // [30:28] - Function number (0-7)
    // [27]    - Block mode (0 for byte mode, 1 for block mode)
    // [26]    - OP Code (0 for R/W to fixed address, 1 for R/W to incrementing address)
    // [25:9]  - Address (17-bit address)
    // [8:0]   - Byte count (for byte mode) or block count (for block mode)
    uint32_t arg = (write << 31) | (functionNumber << 28) | (blockMode << 27) | 
                  (opCode << 26) | ((registerAddress & 0x1FFFF) << 9) | (size & 0x1FF);

    // Set up the basic transfer type for CMD53
    uint32_t xfertyp = SDHC_XFERTYP_CMDINX(53) | SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_BCEN | CMD_RESP_R5; 

    // Add read direction
    if (write == false) {
      xfertyp |=  SDHC_MIX_CTRL_DTDSEL;
    }

    setBlockCountSize(blockMode, functionNumber, size);

    // Debug info
    char cmdInfo[90];
    sprintf(cmdInfo, "CMD: %ld %s  REG: 0x%08lX ARG: 0x%08lX", (xfertyp >> 24) & 0x3F, write ? "WRTE" : "READ", registerAddress, arg);
    if (logOutput) Serial.printf(SER_YELLOW "\n%s\n" SER_RESET, cmdInfo);
    DBG_IRQSTAT();
    
    // Check for command inhibit
    if (waitTimeout(&W4343WCard::isBusyCommandInhibit)) {
      Serial.printf(SER_RED "%s isBusyCommandInhibit\n" SER_RESET, cmdInfo);
      return false;  // Caller will set errorCode.
    }
    
    // Set the argument
    m_psdhc->CMD_ARG = arg;

    // Reset MIX_CTRL
    m_psdhc->MIX_CTRL &= ~SDHC_MIX_CTRL_MASK;
    
    // Set MIX_CTRL for data transfer
    m_psdhc->MIX_CTRL |= xfertyp & SDHC_MIX_CTRL_MASK;

    // Remove flags from transfer type
    xfertyp &= ~SDHC_MIX_CTRL_MASK;

    // Execute command
    m_psdhc->CMD_XFR_TYP = xfertyp;

    // Set up pointer and loop counter to transfer data
    uint32_t * ptr = (uint32_t *)buffer;
    uint16_t wordCount = (size / 4) + (size % 4 > 0 ? 1 : 0);

    // Handle write data transfer
    if (write == true) {
      
      m_transferActive = true;

      if ((m_psdhc->PRES_STATE & SDHC_PRSSTAT_WTA) == false) {
        m_psdhc->PROT_CTRL &= ~SDHC_PROCTL_SABGREQ;
        m_psdhc->PROT_CTRL |= SDHC_PROCTL_CREQ;
      }
      //Set Stop At Block Gap Request
      m_psdhc->PROT_CTRL |= SDHC_PROCTL_SABGREQ;

      uint32_t index = 0;
      while (index < wordCount) {
        // Wait for the buffer write enable flag
        while (0 == (m_psdhc->PRES_STATE & SDHC_PRSSTAT_BWEN)) {}
        uint32_t writeCount = (wordCount - index) > FIFO_WML ? FIFO_WML : wordCount - index;
        for (uint32_t i = 0; i < writeCount; i++) {
          m_psdhc->DATA_BUFF_ACC_PORT = ptr[index];
          if (logOutput) Serial.printf("Write: 0x%08X\n", ptr[index]);
          index++;
        }
      }

      m_transferActive = false;
      if (logOutput) Serial.println("Write done");
    }
 
    // Handle read data transfer
    if (write == false) {

      // Read data from the SDIO card
      uint32_t index = 0;
      while (index < wordCount) {
        // Wait for the buffer read enable flag
        while (0 == (m_psdhc->PRES_STATE & SDHC_PRSSTAT_BREN)) {}
        uint32_t readCount = (wordCount - index) > FIFO_WML ? FIFO_WML : wordCount - index;
        for (uint32_t i = 0; i < readCount; i++) {
          ptr[index] = m_psdhc->DATA_BUFF_ACC_PORT;
          if (logOutput) Serial.printf("Read: 0x%08X\n", ptr[index]);
          index++;
        }
      }

      //TODO test code - dummy read as occasionally more data is returned than expected Likely occurs when asking
      //TODO for more bytes than a register wants to give up. Safe to keep for now, during development.
      while (m_psdhc->PRES_STATE & SDHC_PRSSTAT_BREN) {
         Serial.printf(SER_ERROR "Dummy: 0x%08X\n" SER_RESET, m_psdhc->DATA_BUFF_ACC_PORT);
      }

      if (logOutput) Serial.println("Read done");
    }
    
    // Wait for transfer completion
    if (waitTimeout(&W4343WCard::isBusyTransferComplete)) {
      Serial.printf(SER_RED "%s isBusyTransferComplete\n" SER_RESET, cmdInfo);
      return false;  // Caller will set errorCode.
    } 

    // Wait for command completion
    if (waitTimeout(&W4343WCard::isBusyCommandComplete)) {
      Serial.printf(SER_RED "%s isBusyCommandComplete\n" SER_RESET, cmdInfo);
      return sdError(SD_CARD_ERROR_READ_TIMEOUT);
    }  

    // Check for errors
    m_irqstat = m_psdhc->INT_STATUS;
    m_psdhc->INT_STATUS = m_irqstat;

    return_value = (m_irqstat & SDHC_IRQSTAT_CC) && !(m_irqstat & SDHC_IRQSTAT_CMD_ERROR);
    if (logOutput) {
      printResponse(return_value);
      Serial.println(!return_value ? SER_RED "cmd failed" SER_RESET : "cmd complete");
    }

    return return_value;
}

// Set backplane window, don't set if already set
void W4343WCard::setBackplaneWindow(uint32_t addr)
{
  static uint32_t prevAddr = 0;
  
  addr &= SB_WIN_MASK;

  if (addr != prevAddr) {
    cardCMD52_write(SD_FUNC_BAK, BAK_WIN_ADDR_REG, addr >> 8);
    cardCMD52_write(SD_FUNC_BAK, BAK_WIN_ADDR_REG + 1, addr >> 16);
    cardCMD52_write(SD_FUNC_BAK, BAK_WIN_ADDR_REG + 2, addr >> 24);
    prevAddr = addr;
    Serial.printf(SER_TRACE "Backplane Window set to 0x%08X\n" SER_RESET, addr);
  }
}

// Set backplane window, and return offset within window
uint32_t W4343WCard::setBackplaneWindow_retOffset(uint32_t addr)
{
  setBackplaneWindow(addr);
  return(addr & SB_ADDR_MASK);
}

// Read a 32-bit value via the backplane window
uint32_t W4343WCard::backplaneWindow_read32(uint32_t addr, uint32_t *valp)
{
  u32Data u32d;
  uint32_t n;

  setBackplaneWindow(addr);
  n = cardCMD53_read(SD_FUNC_BAK, addr | SB_32BIT_WIN, u32d.bytes, 4, false);
  *valp = u32d.uint32;
  return n;
}

// Write a 32-bit value via the backplane window
uint32_t W4343WCard::backplaneWindow_write32(uint32_t addr, uint32_t val)
{
  u32Data u32d={.uint32=val};

  setBackplaneWindow(addr);
  return cardCMD53_write(SD_FUNC_BAK, addr | SB_32BIT_WIN, u32d.bytes, 4, false);
}

bool W4343WCard::uploadFirmware(size_t firmwareSize, uintptr_t source)
{
  uint32_t nBytesSent = 0;
  uint32_t len = 0;
  uint32_t addr;

  Serial.printf(SER_CYAN "\nUploading firmware data\n" SER_RESET);

  while (nBytesSent < firmwareSize) {

    addr = setBackplaneWindow_retOffset(nBytesSent);
    len = MIN(64, firmwareSize - nBytesSent);

    //TODO implement block sends, use byte mode send for now
    cardCMD53_write(SD_FUNC_BAK, SB_32BIT_WIN + addr, (uint8_t *)source + nBytesSent, len, false);
    nBytesSent += len;
  }
  Serial.printf(SER_CYAN "\nUploaded firmware, %ld of %ld bytes\n" SER_RESET, nBytesSent, firmwareSize);
  return true;
}

bool W4343WCard::uploadNVRAM(size_t nvRAMSize, uintptr_t source)
{
  uint32_t nBytesSent = 0;
  uint32_t len = 0;
  //I believe the offset value 0xfd54 in Zero code is too co-incidental equal to 0x10000 - nvRAMSize, 
  //so calc this. Offset 4 bytes earlier, to leave room for size calculation
  uint32_t offset = (0x10000 - nvRAMSize) - 4;

  Serial.printf(SER_CYAN "\nUploading NVRAM data\n" SER_RESET);

  setBackplaneWindow(0x078000);
  while (nBytesSent < nvRAMSize)
  {
      len = MIN(nvRAMSize - nBytesSent, SD_BAK_BLK_BYTES);
      cardCMD53_write(SD_FUNC_BAK, offset + nBytesSent, (uint8_t *)source + nBytesSent, len, false);
      nBytesSent += len;
  }

  //Calculate size, write at end of section
  u32Data u32d;
  u32d.uint32 = ((~(nvRAMSize / 4) & 0xffff) << 16) | (nvRAMSize / 4);
  cardCMD53_write(SD_FUNC_BAK, 0x10000 - 4, u32d.bytes, 4, true);

  Serial.printf(SER_CYAN "\nUploaded NVRAM, %ld of %ld bytes\n" SER_RESET, nBytesSent, nvRAMSize);
  return true;
}

void W4343WCard::printResponse(bool return_value)
{
  Serial.printf("RSP: 0x%4.4X  0x%4.4X  0x%4.4X  0x%4.4X   RET: 0x%02X\n",m_psdhc->CMD_RSP0, m_psdhc->CMD_RSP1, m_psdhc->CMD_RSP2, m_psdhc->CMD_RSP3, return_value);
}

/////////////////////////////
// WLAN interaction functions
/////////////////////////////


IOCTL_MSG ioctl_txmsg, ioctl_rxmsg;
int txglom;
uint16_t ioctl_reqid=0;
uint8_t event_mask[EVENT_MAX / 8];
EVT_STR *current_evts;
char ioctl_event_hdr_fields[] =  
    "2:len 2: 1:seq 1:chan 1: 1:hdrlen 1:flow 1:credit";
#define MAX_EVENT_STATUS 16
const char * event_status[MAX_EVENT_STATUS] = {
    "SUCCESS","FAIL","TIMEOUT","NO_NETWORK","ABORT","NO_ACK",
    "UNSOLICITED","ATTEMPT","PARTIAL","NEWSCAN","NEWASSOC",
    "11HQUIET","SUPPRESS","NOCHANS","CCXFASTRM","CS_ABORT" };

// Event handling
#define DISP_BLOCKLEN       32
uint8_t eventbuff[1600];

IOCTL_EVENT_HDR ieh;
ETH_EVENT_FRAME *eep = (ETH_EVENT_FRAME *)eventbuff;

EVT_STR escan_evts[] = ESCAN_EVTS;
// Event groups
EVT_STR join_evts[]=JOIN_EVTS, no_evts[]=NO_EVTS;
// Event field displays
char eth_hdr_fields[]   = "6:dest 6:srce 2;type";
char event_hdr_fields[] = "2;sub 2;len 1: 3;oui 2;usr";
char event_msg_fields[] = "2;ver 2;flags 4;type 4;status 4;reason 4:auth 4;dlen 6;addr 18:";

// Network scan parameters
SCAN_PARAMS scan_params = {
    .version=1, .action=1, .sync_id=0x1234, .ssidlen=0, .ssid={0}, 
    .bssid={0xff,0xff,0xff,0xff,0xff,0xff}, .bss_type=2,
    .scan_type=SCANTYPE_PASSIVE, .nprobes=-1, .active_time=-1,
    .passive_time=-1, .home_time=-1, 
#if SCAN_CHAN == 0
    .nchans=14, .nssids=0, 
    .chans={{1,0x2b},{2,0x2b},{3,0x2b},{4,0x2b},{5,0x2b},{6,0x2b},{7,0x2b},
      {8,0x2b},{9,0x2b},{10,0x2b},{11,0x2b},{12,0x2b},{13,0x2b},{14,0x2b}},
#else
    .nchans=1, .nssids=0, .chans={{SCAN_CHAN,0x2b}}, .ssids={{0}}
#endif
};

uint8_t resp[256] = {0}, eth[7]={0};
escan_result *erp = (escan_result *)eventbuff;
bool clkval = false;
bool ledon = false;

void W4343WCard::getMACAddress()
{
  uint8_t eth[7]={0};

  uint32_t n = ioctl_get_data("cur_etheraddr", 0, eth, 6);

  Serial.printf("%sMAC address ", n > 0 ? SER_GREEN : SER_RED);
  if (n > 0) {
    for (uint8_t i = 0; i < 6; i++) {
      Serial.printf("%s%02X", i ? ":" : "", eth[i]);
    }
  }
  Serial.println(SER_RESET);
}

void W4343WCard::printMACAddress(uint8_t * data)
{
  for (uint8_t i = 0; i < 6; i++) {
    Serial.printf("%s%02X", i ? ":" : "", data[i]);
  }
}

// Display SSID
void W4343WCard::printSSID(uint8_t * data)
{
    int i = *data++;

    if (i == 0 || *data == 0) {
      Serial.printf("[hidden]");
    } else if (i <= SSID_MAXLEN) {
      Serial.printf(SER_GREEN);
      while (i-- > 0) {
        char c = static_cast<char>(*data++); 
        Serial.print(c);
      }
      Serial.printf(SER_RESET);
    } else {
      Serial.printf("[invalid length %u]", i);
   }
}

void W4343WCard::getFirmwareVersion()
{
  uint32_t n = ioctl_get_data("ver", 0, resp, sizeof(resp));
  Serial.printf("\n%sFirmware %s\n" SER_RESET, (n ? SER_GREEN : SER_RED), (n ? (char *)resp : "not responding"));
}

void W4343WCard::ScanNetworks()
{
  uint32_t val = 0;
  
  ioctl_wr_int32(IOCTL_SET_SCAN_CHANNEL_TIME, 0, SCAN_CHAN_TIME);

  if (!ioctl_wr_int32(WLC_UP, 200, 0)) {
    Serial.printf(SER_RED "\nWiFi CPU not running\n" SER_RESET);
    return;
  } else {
    Serial.printf(SER_GREEN "\nWiFi CPU running\n" SER_RESET);
  }

  backplaneWindow_write32(SB_INT_STATUS_REG, val);
  cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, (uint8_t *)resp, 64);

  if (ioctl_enable_evts(escan_evts) == true) {
    Serial.printf(SER_TRACE "\nEvents enabled\n" SER_RESET);
  } else {
    Serial.printf(SER_RED "\nEvents not enabled\n" SER_RESET);
    return;
  }

  if (ioctl_set_data("escan", 0, &scan_params, sizeof(scan_params)) == true) {
    Serial.printf(SER_TRACE "\nSet data escan\n" SER_RESET);
  } else {
    Serial.printf(SER_RED "\nFailed to set data escan\n" SER_RESET);
    return;
  }

  while (1) {
//    delay(1000);
    uint32_t n = ioctl_get_event(&ieh, eventbuff, sizeof(eventbuff));
    
    if (n > sizeof(escan_result)) {
      printMACAddress((uint8_t *)&erp->event.whd_event.addr);
      Serial.printf(" %d ", __builtin_bswap16(erp->escan.bss_info->chanspec));
      printSSID(&erp->escan.bss_info->SSID_len);
      Serial.printf("\n");
    }
  }
}

//----------------------------------------------------------------------
// JoinNetworks() added 02-20-25 WW
//----------------------------------------------------------------------
void W4343WCard::JoinNetworks(const char *ssID, const char *passphrase, int security) {
    int ticks=0, ledon=0, n, startime=micros();
    
    // Process SSID
	wlc_ssid_t ssid;
	ssid.SSID_len = strlen(ssID);
	strcpy((char *)ssid.SSID, ssID);
    // Process PASSWORD
	wsec_pmk_t wsec_pmk;
	wsec_pmk.key_len = strlen(passphrase);
	wsec_pmk.flags = WSEC_PASSPHRASE;
	strcpy((char *)wsec_pmk.key, passphrase);
    
    cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, resp, 64);
    n = ioctl_get_data("cur_etheraddr", 0, eth, 6);
    Serial.printf("MAC address ");
    if (n)
      printMACAddress(eth);
    else
      printf("unavailable");
    n = ioctl_get_data("ver", 0, resp, sizeof(resp));
    Serial.printf("\nFirmware %s\n", (n ? (char *)resp : "not responding"));
    if (!ioctl_set_data("country", 100, &country_struct, sizeof(country_struct)))
        Serial.printf("Can't set country\n");
    if (!ioctl_wr_int32(WLC_UP, 200, 0)) Serial.printf("WiFi CPU not running\n");

    ioctl_enable_evts(no_evts);
    CHECK(ioctl_wr_int32, WLC_SET_INFRA, 50, 1);
    CHECK(ioctl_wr_int32, WLC_SET_AUTH, 0, 0);
    if(security != 0) {
      CHECK(ioctl_wr_int32, WLC_SET_WSEC, 0, security==2 ? 6 : 2);
      CHECK(ioctl_set_intx2, "bsscfg:sup_wpa", 0, 0, 1);
      CHECK(ioctl_set_intx2, "bsscfg:sup_wpa2_eapver", 0, 0, -1);
      CHECK(ioctl_set_intx2, "bsscfg:sup_wpa_tmo", 0, 0, 2500);
      CHECK(ioctl_wr_data, WLC_SET_WSEC_PMK, 0, &wsec_pmk, sizeof(wsec_pmk));
      CHECK(ioctl_wr_int32, WLC_SET_WPA_AUTH, 0, security==2 ? 0x80 : 4);
    } else {
      CHECK(ioctl_wr_int32, WLC_SET_WSEC, 0, 0);
      CHECK(ioctl_wr_int32, WLC_SET_WPA_AUTH, 0, 0);
    }
    ioctl_enable_evts(join_evts);
    CHECK(ioctl_wr_data, WLC_SET_SSID, 100, &ssid, sizeof(ssid));

    while (1)
    {
// This area is unfinished!!!! More work needed!!!
delay(1000);  // Temporary delay to see what's going on...
//        delayMicroseconds(SD_CLK_DELAY);
//        if (ustimeout(&ticks, 20000))
//        {
//            digitalWrite(LED_PIN, ledon = !ledon);
//            if (!ledon)
//            {
//                Serial.printf(".");
//            }
//            else
//            {
                if ((n=ioctl_get_event(&ieh, eventbuff, sizeof(eventbuff))) > 0)
                {
                    Serial.printf("\n%2.3f ", (micros() - startime) / 1e6);
                    disp_fields(&ieh, ioctl_event_hdr_fields, n);
                    Serial.printf("\n");
                    disp_bytes((uint8_t *)&ieh, sizeof(ieh));
                    Serial.printf("\n");
                    disp_fields(&eep->eth_hdr, eth_hdr_fields, sizeof(eep->eth_hdr));
                    if (SWAP16(eep->eth_hdr.ethertype) == 0x886c)
                    {
                        disp_fields(&eep->event.hdr, event_hdr_fields, sizeof(eep->event.hdr));
                        Serial.printf("\n");
                        disp_fields(&eep->event.msg, event_msg_fields, sizeof(eep->event.msg));
                        Serial.printf("%s %s", ioctl_evt_str(SWAP32(eep->event.msg.event_type)),
                               ioctl_evt_status_str(SWAP32(eep->event.msg.status)));
                    }
                    Serial.printf("\n");
                    disp_block(eventbuff, n);
                    Serial.printf("\n");

                }
//            }
//        }
    }
}
//----------------------------------------------------------------------

///////////////////////////
///////////////////////////
///////////////////////////

// Get event data, return data length excluding header
uint32_t W4343WCard::ioctl_get_event(IOCTL_EVENT_HDR *hp, uint8_t *data, int maxlen)
{
    int n=0, dlen=0, blklen;

    hp->len = 0;
    bool res = cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, (uint8_t *)hp, sizeof(IOCTL_EVENT_HDR), false);
    //Serial.printf(SER_CYAN "Init Read: %ld, IOCTL_EVENT_HDR struct size: %ld\n" SER_RESET, hp->len, sizeof(IOCTL_EVENT_HDR));
    if (res == true && hp->len > sizeof(IOCTL_EVENT_HDR) && hp->notlen > 0 && hp->len == (hp->notlen^0xffff))
    {
        dlen = hp->len - sizeof(IOCTL_EVENT_HDR);
        while (n < dlen && n < maxlen)
        {
            blklen = MIN(MIN(maxlen - n, hp->len - n), IOCTL_MAX_BLKLEN);
            cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, (uint8_t *)(&data[n]), blklen, false);
            n += blklen;
        }
        //Read and discard remaining bytes over maxlen
        while (n < dlen)
        {
            blklen = MIN(hp->len - n, IOCTL_MAX_BLKLEN);
            cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, 0, blklen, false);
            n += blklen;
        }
    }
    return dlen > maxlen ? maxlen : dlen;
}

// Enable events
int W4343WCard::ioctl_enable_evts(EVT_STR *evtp)
{
    current_evts = evtp;
    memset(event_mask, 0, sizeof(event_mask));
    while (evtp->num >= 0)
    {
        if (evtp->num / 8 < (int32_t)sizeof(event_mask))
            SET_EVENT(event_mask, evtp->num);
        evtp++;
    }
    return ioctl_set_data("event_msgs", 0, event_mask, sizeof(event_mask));
}

// IOCTL write with integer parameter
int W4343WCard::ioctl_wr_int32(int cmd, int wait_msec, int val)
{
  u32Data u32 = {.uint32=(uint32_t)val};

  return ioctl_cmd(cmd, 0, wait_msec, 1, u32.bytes, 4);
}

// Get data block from IOCTL variable
int W4343WCard::ioctl_get_data(const char *name, int wait_msec, uint8_t *data, int dlen)
{
  return ioctl_cmd(WLC_GET_VAR, name, wait_msec, 0, data, dlen);
}

// Set data block in IOCTL variable
int W4343WCard::ioctl_set_data(const char *name, int wait_msec, void *data, int len)
{
    return ioctl_cmd(WLC_SET_VAR, name, wait_msec, 1, data, len);
}

//----------------------------------------------------------------------
// IOCTL write data - added 02-20-25 WW
int W4343WCard::ioctl_wr_data(int cmd, int wait_msec, void *data, int len)
{
    return(ioctl_cmd(cmd, 0, wait_msec, 1, data, len));
}

//----------------------------------------------------------------------
// IOCTL read data - added 02-20-25 WW
int W4343WCard::ioctl_rd_data(int cmd, int wait_msec, void *data, int len)
{
    return(ioctl_cmd(cmd, 0, wait_msec, 0, data, len));
}
//----------------------------------------------------------------------

// Do an IOCTL transaction, get response, optionally waiting for it
int W4343WCard::ioctl_cmd(int cmd, const char *name, int wait_msec, int wr, void *data, int dlen)
{
  static uint8_t txseq=1;
  IOCTL_MSG *msgp = &ioctl_txmsg, *rsp = &ioctl_rxmsg;
  IOCTL_CMD *cmdp = txglom ? &msgp->glom_cmd.cmd : &msgp->cmd;
  int ret=0, namelen = name ? strlen(name)+1 : 0;
  int txdlen = wr ? namelen + dlen : MAX(namelen, dlen);
  int hdrlen = cmdp->data - (uint8_t *)&ioctl_txmsg;
  int txlen = ((hdrlen + txdlen + 3) / 4) * 4; //, rxlen;
  uint32_t val = 0;

  // Prepare IOCTL command
  memset(msgp, 0, sizeof(ioctl_txmsg));
  memset(rsp, 0, sizeof(ioctl_rxmsg));
  msgp->notlen = ~(msgp->len = hdrlen+txdlen);
  if (txglom)
  {
    msgp->glom_cmd.glom_hdr.len = hdrlen + txdlen - 4;
    msgp->glom_cmd.glom_hdr.flags = 1;
  }
  cmdp->seq = txseq++;
  cmdp->hdrlen = txglom ? 20 : 12;
  cmdp->cmd = cmd;
  cmdp->outlen = txdlen;
  cmdp->flags = ((uint32_t)++ioctl_reqid << 16) | (wr ? 2 : 0);
  if (namelen)
    memcpy(cmdp->data, name, namelen);
  if (wr)
    memcpy(&cmdp->data[namelen], data, dlen);
  // Send IOCTL command
  cardCMD53_write(SD_FUNC_RAD, SB_32BIT_WIN, (uint8_t *)msgp, txlen, false);
  ioctl_wait(IOCTL_WAIT_USEC);
  while (wait_msec >= 0 && ret == 0) {
    // Wait for response to be available
    wait_msec -= IOCTL_POLL_MSEC;
    backplaneWindow_read32(SB_INT_STATUS_REG, &val);
    // If response is waiting..
    if (val & 0xff) {
      // ..request response
      backplaneWindow_write32(SB_INT_STATUS_REG, val);
      // Fetch response
      ret = cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, (uint8_t *)rsp, txlen, false);

      // Discard response if not matching request
      if ((rsp->cmd.flags>>16) != ioctl_reqid) {
        ret = 0;
      }
      // Exit if error response
      if (ret && (rsp->cmd.flags & 1))
      {
        ret = 0;
        break;
      }
      // If OK, copy data to buffer
      if (ret && !wr && data && dlen) {
        memcpy(data, rsp->cmd.data, dlen);
      }
    }
    // If no response, wait
    else {
      Serial.printf("No respsone, wait....\n");
      delay(IOCTL_POLL_MSEC);
    }
  }
  return ret;
}

// Wait until IOCTL command has been processed. Pins need to be in GPIO mode for this read to work
bool W4343WCard::ioctl_wait(int usec)
{
  bool ready = false;
  uint32_t startMicros = micros();
  //TODO restore wait time before timeout, or switch to interrupt - currently takes significantly longer than the requested timeout
  while (ready == false) {// && !(micros() > startMicros + (usec * 2))) {
    ready = fUseSDIO2 == false ? (GPIO3_PSR & (1 << 15)) ? false : true : (GPIO1_PSR & (1 << 21)) ? false : true;
  }

  Serial.printf("%s Ready in %lduS, expected %lduS\n" SER_RESET, ready == true ? SER_MAGENTA "Is": SER_RED "Not", micros()- startMicros, usec);
  return ready;
}

////////////////////////
// End IRW new functions
////////////////////////

//------------------------------------------------------------------------------
bool W4343WCard::cardCommand(uint32_t xfertyp, uint32_t arg) {
  // Debug string
  char cmdInfo[90];
  sprintf(cmdInfo, "CMD: %ld ARG: 0x%08lX", (xfertyp >> 24) & 0x3F,  arg);
  
  Serial.printf(SER_YELLOW "\n%s\n" SER_RESET, cmdInfo);
  DBG_IRQSTAT();

  if (waitTimeout(&W4343WCard::isBusyCommandInhibit)) {
    Serial.printf("cardCommand(%x, %x) error isBusyCommandInhibit\n", xfertyp, arg);
    return false;  // Caller will set errorCode.
  }
  m_psdhc->CMD_ARG = arg;
  // Set MIX_CTRL if data transfer.
  if (xfertyp & SDHC_XFERTYP_DPSEL) {
    m_psdhc->MIX_CTRL &= ~SDHC_MIX_CTRL_MASK;
    m_psdhc->MIX_CTRL |= xfertyp & SDHC_MIX_CTRL_MASK; //Enables DMA based on SDHC_MIX_CTRL_DMAEN set in xfertyp (DATA_READ_DMA, DATA_WRITE_DMA)
  }
  xfertyp &= ~SDHC_MIX_CTRL_MASK;
  m_psdhc->CMD_XFR_TYP = xfertyp;  // Execute command

  if (waitTimeout(&W4343WCard::isBusyCommandComplete)) {
    Serial.printf("cardCommand(%x, %x) error isBusyCommandComplete\n", xfertyp, arg);
    return false;  // Caller will set errorCode.
  }
  m_irqstat = m_psdhc->INT_STATUS;
  m_psdhc->INT_STATUS = m_irqstat;

  bool return_value = (m_irqstat & SDHC_IRQSTAT_CC) && !(m_irqstat & SDHC_IRQSTAT_CMD_ERROR);
  printResponse(return_value);
  return return_value;      
}

//------------------------------------------------------------------------------
void W4343WCard::enableDmaIrs() {
  m_dmaBusy = true;
  m_irqstat = 0;
}
//------------------------------------------------------------------------------
void W4343WCard::initSDHC() {
  // initialize Hardware registers and this ointer
  if (fUseSDIO2 == false) {
    s_pSdioCards[0] = this; 
    m_psdhc = (IMXRT_USDHC_t*)IMXRT_USDHC1_ADDRESS;   
  } else {
    s_pSdioCards[1] = this; 
    m_psdhc = (IMXRT_USDHC_t*)IMXRT_USDHC2_ADDRESS; 
  }

  initClock();

  // Disable GPIO
  enableSDIO(false);

  m_psdhc->MIX_CTRL |= 0x80000000;

  // Reset SDHC. Use default Water Mark Level of 16.
  m_psdhc->SYS_CTRL |= SDHC_SYSCTL_RSTA | SDHC_SYSCTL_SDCLKFS(0x80);

  while (m_psdhc->SYS_CTRL & SDHC_SYSCTL_RSTA) {
  }

  // Set initial SCK rate.
  setSdclk(SD_MAX_INIT_RATE_KHZ);

  enableSDIO(true);

  // Enable desired IRQSTAT bits.
  m_psdhc->INT_STATUS_EN = SDHC_IRQSTATEN_MASK;

  if (fUseSDIO2 == false) {
    attachInterruptVector(IRQ_SDHC1, sdISR);
    NVIC_SET_PRIORITY(IRQ_SDHC1, 6*16);
    NVIC_ENABLE_IRQ(IRQ_SDHC1);
  } else {
    attachInterruptVector(IRQ_SDHC2, sdISR2);
    NVIC_SET_PRIORITY(IRQ_SDHC2, 6*16);
    NVIC_ENABLE_IRQ(IRQ_SDHC2);    
  }

  // Send 80 clocks to card.
  m_psdhc->SYS_CTRL |= SDHC_SYSCTL_INITA;
  while (m_psdhc->SYS_CTRL & SDHC_SYSCTL_INITA) {
  }
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyCommandComplete() {
  return !(m_psdhc->INT_STATUS & (SDHC_IRQSTAT_CC | SDHC_IRQSTAT_CMD_ERROR));
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyCommandInhibit() {
  return m_psdhc->PRES_STATE & SDHC_PRSSTAT_CIHB;
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyDat() {
  return m_psdhc->PRES_STATE & (1 << 24) ? false : true;
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyFifoRead() {
  return !(m_psdhc->PRES_STATE & SDHC_PRSSTAT_BREN);
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyFifoWrite() {
  return !(m_psdhc->PRES_STATE & SDHC_PRSSTAT_BWEN);
}
//------------------------------------------------------------------------------
bool W4343WCard::isBusyTransferComplete() {
  return !(m_psdhc->INT_STATUS & (SDHC_IRQSTAT_TC | SDHC_IRQSTAT_ERROR));
}
//------------------------------------------------------------------------------
void W4343WCard::setSdclk(uint32_t kHzMax) {
  const uint32_t DVS_LIMIT = 0X10;
  const uint32_t SDCLKFS_LIMIT = 0X100;
  uint32_t dvs = 1;
  uint32_t sdclkfs = 1;
  uint32_t maxSdclk = 1000*kHzMax;
  uint32_t baseClk = baseClock();

  while ((baseClk/(sdclkfs*DVS_LIMIT) > maxSdclk) && (sdclkfs < SDCLKFS_LIMIT)) {
    sdclkfs <<= 1;
  }
  while ((baseClk/(sdclkfs*dvs) > maxSdclk) && (dvs < DVS_LIMIT)) {
    dvs++;
  }
  m_sdClkKhz = baseClk/(1000*sdclkfs*dvs);
  sdclkfs >>= 1;
  dvs--;

  // Change dividers.
  uint32_t sysctl = m_psdhc->SYS_CTRL & ~(SDHC_SYSCTL_DTOCV_MASK
                    | SDHC_SYSCTL_DVS_MASK | SDHC_SYSCTL_SDCLKFS_MASK);

  m_psdhc->SYS_CTRL = sysctl | SDHC_SYSCTL_DTOCV(0x0E) | SDHC_SYSCTL_DVS(dvs)
                | SDHC_SYSCTL_SDCLKFS(sdclkfs);

  // Wait until the SDHC clock is stable.
  while (!(m_psdhc->PRES_STATE & SDHC_PRSSTAT_SDSTB)) {
  }

}
//------------------------------------------------------------------------------
// Return true if timeout occurs.
bool W4343WCard::waitTimeout(pcheckfcn fcn) {
  uint32_t m = micros();
  while ((this->*fcn)()) {
    if ((micros() - m) > BUSY_TIMEOUT_MICROS) {
      return true;
    }
  }
  return false;  // Caller will set errorCode.
}
//------------------------------------------------------------------------------
bool W4343WCard::waitTransferComplete() {
  if (!m_transferActive) {
    return true;
  }
  uint32_t m = micros();
  bool timeOut = false;
  while (isBusyTransferComplete()) {
    if ((micros() - m) > BUSY_TIMEOUT_MICROS) {
      timeOut = true;
      break;
    }
  }
  m_transferActive = false;
  m_irqstat = m_psdhc->INT_STATUS;
  m_psdhc->INT_STATUS = m_irqstat;
  if (timeOut || (m_irqstat & SDHC_IRQSTAT_ERROR)) {
    return sdError(SD_CARD_ERROR_TRANSFER_COMPLETE);
  }
  return true;
}

bool W4343WCard::SDIOEnableFunction(uint8_t functionEnable)
{
  uint8_t readResponse;

  //Read existing register
  cardCMD52_read(SD_FUNC_BUS, BUS_IOEN_REG, &readResponse);

  //Set function enable
  readResponse |= functionEnable;

  //Write back register
  cardCMD52_write(SD_FUNC_BUS, BUS_IOEN_REG, readResponse);

  //Verify
  for (uint8_t i = 0; i < 100; i++) {
    cardCMD52_read(SD_FUNC_BUS, BUS_IOEN_REG, &readResponse);
    if (readResponse & functionEnable) {
      Serial.printf(SER_TRACE "SDIO function mask 0x%02X enabled\n" SER_RESET, functionEnable);
      return true;
    } 
    delay(1);
  }

  Serial.printf(SER_ERROR "SDIO function mask 0x%02X not enabled\n" SER_RESET, functionEnable);
  return false;
}

bool W4343WCard::SDIODisableFunction(uint8_t functionEnable)
{
  uint8_t readResponse;

  //Read existing register
  cardCMD52_read(SD_FUNC_BUS, BUS_IOEN_REG, &readResponse);

  //Set function disable
  readResponse &= ~functionEnable;

  //Write back register
  cardCMD52_write(SD_FUNC_BUS, BUS_IOEN_REG, readResponse);

  //TODO validate written?
  Serial.printf(SER_TRACE "SDIO function mask 0x%02X disabled\n" SER_RESET, functionEnable);
  return true;
}

//bcmsdh.c, line 95, brcmf_sdiod_intr_register
bool W4343WCard::configureOOBInterrupt()
{
  uint8_t readResponse;
  Serial.println(SER_TRACE "\nConfiguring WL_IRQ OOB" SER_RESET);
  // Must configure BUS_INTEN_REG to enable irq
  cardCMD52_read(SD_FUNC_BUS, BUS_INTEN_REG, &readResponse);
  readResponse |= SDIO_CCCR_IEN_FUNC0 | SDIO_CCCR_IEN_FUNC1 | SDIO_CCCR_IEN_FUNC2;
  cardCMD52_write(SD_FUNC_BUS, BUS_INTEN_REG, readResponse);

  // Redirect, configure and enable IO for interrupt signal
  readResponse = SDIO_CCCR_BRCM_SEPINT_MASK | SDIO_CCCR_BRCM_SEPINT_OE;
  cardCMD52_write(SD_FUNC_BUS, BUS_SEP_INT_CTL, readResponse);

  //TODO any validation?
  return true;
}

void W4343WCard::onWLIRQInterruptHandler()
{
  dataISRReceived = true;
  //Yeah yeah, no Serial in ISRs, I know....
  Serial.println(SER_MAGENTA "WL_IRQ OOB Interrupt" SER_RESET);
}

void W4343WCard::onDataInterruptHandler()
{
  dataISRReceived = true;
  //Yeah yeah, no Serial in ISRs, I know....
  Serial.println(SER_MAGENTA "DAT1 IB Interrupt" SER_RESET);
}

//==============================================================================
// Start of W4343WCard member functions.
//==============================================================================
bool W4343WCard::begin(bool useSDIO2, int8_t wlOnPin, int8_t wlIrqPin, int8_t extLPOPin) 
{
  uint32_t kHzSdClk;
  m_curState = IDLE_STATE;
  m_initDone = false;
  m_errorCode = SD_CARD_ERROR_NONE;
  m_highCapacity = false;
  m_version2 = false;
  fUseSDIO2 = useSDIO2;

  uint8_t readResponse;
  u32Data u32d;
  uint8_t data[520];
  
  Serial.printf("==========================\nW4343WCard::begin: %s\n==========================\n", fUseSDIO2 ? "SDIO2" : "SDIO");
 
  initSDHC();

  ////////////
  //Setup pins
  ////////////

  ///////////////////
  //In-band interrupt
  ///////////////////
  //Attach in-band interrupt to DAT1 (GPIO mode only) - not in use, yet
  //TODO pin 34 is DAT1 on DB5. Need to change depending on device. Eventually use lower level attachment, avoiding pin
  //#define DAT1_INTERRUPT_PIN 34
  //Serial.printf(SER_TRACE "Attaching IB interrupt to pin %d\n" SER_RESET, DAT1_INTERRUPT_PIN);
  //pinMode(DAT1_INTERRUPT_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(DAT1_INTERRUPT_PIN), onDataInterruptHandler, FALLING);

  //////////////////////////////////
  //out-of-band interrupt on INT pin
  //////////////////////////////////
  if (wlIrqPin > -1) {
    Serial.printf(SER_TRACE "Attaching OOB interrupt to pin %d\n" SER_RESET, wlIrqPin);
    pinMode(wlIrqPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(wlIrqPin), onWLIRQInterruptHandler, FALLING);
  }

  ////////////////////
  //External LPO clock
  ////////////////////
  #if defined(USE_EXTERNAL_LPO)
  if (extLPOPin > -1) {
    pinMode(extLPOPin, OUTPUT);    // Set the pin as output
    
    analogWriteFrequency(extLPOPin, 32768);  // Set the frequency to 32.768 kHz
    analogWrite(extLPOPin, 128);             // 50% duty cycle 
  }
  #endif

  ///////////
  //WL_ON pin
  ///////////
  pinMode(wlOnPin, OUTPUT);  // Pull high to activate WLAN 
  digitalWriteFast(wlOnPin, HIGH); // Enable WLAN
  delay(185);                      // Delay to allow reset to complete

  ////////////////////////
  //Sequence is heavily influenced by the Pi Zero blog. Enhancements made to the flow by studying
  //Broadcom Full MAC and other drivers - Pi Zero code is a playback of a 'recording' of SDIO commands,
  //and shows apparently unused reads before writes that, in driver code, show that actually the data read
  //is manipulated, and written back.
  ////////////////////////

  //CMD 0
  cardCommand(CMD0_XFERTYP, 0);
  //CMD 5
  cardCommand(CMD5_XFERTYP, 0);
  //CMD 5
  //cardCommand(CMD5_XFERTYP, 0x200000); // Not needed
  //CMD 3
  cardCommand(CMD3_XFERTYP, 0);
  m_rca = m_psdhc->CMD_RSP0 & 0xFFFF0000;
  //CMD 7
  cardCommand(CMD7_XFERTYP, m_rca);
  
  //Set function block sizes. CYW4343W datasheet p16 - F0/1/2 - 32/64/512
  cardCMD52_write(SD_FUNC_BUS, BUS_SDIO_BLKSIZE_REG, SD_BUS_BLK_BYTES & 0xFF);
  cardCMD52_write(SD_FUNC_BUS, BUS_SDIO_BLKSIZE_REG + 1, SD_BUS_BLK_BYTES >> 8);

  //Set backplane block size
  cardCMD52_write(SD_FUNC_BUS, BUS_BAK_BLKSIZE_REG, SD_BAK_BLK_BYTES & 0xFF);
  cardCMD52_write(SD_FUNC_BUS, BUS_BAK_BLKSIZE_REG + 1, SD_BAK_BLK_BYTES >> 8);
  
  //Set radio block size
  cardCMD52_write(SD_FUNC_BUS, BUS_RAD_BLKSIZE_REG, SD_RAD_BLK_BYTES & 0xFF);
  cardCMD52_write(SD_FUNC_BUS, BUS_RAD_BLKSIZE_REG + 1, SD_RAD_BLK_BYTES >> 8);

  //Device must support high-speed mode
  cardCMD52_read(SD_FUNC_BUS, BUS_SPEED_CTRL_REG, &readResponse);
  if (readResponse & 1) {
    //Set card bus high speed interface
    cardCMD52_write(SD_FUNC_BUS, BUS_SPEED_CTRL_REG, readResponse | 2); // Zero set to 0x03
    Serial.println(SER_TRACE "\nEnabled 4343W bus high speed interface" SER_RESET);
  }

  //Set the card bus to 4-bits
  cardCMD52_read(SD_FUNC_BUS, BUS_BI_CTRL_REG, &readResponse);
  cardCMD52_write(SD_FUNC_BUS, BUS_BI_CTRL_REG, (readResponse & ~3) | 2); // Zero set to 0x42

  //Enable I/O 
  if (SDIOEnableFunction(SD_FUNC_BAK_EN) == false) {
    return false;
  }

  //Verify I/O is ready
  cardCMD52_read(SD_FUNC_BUS, BUS_IORDY_REG, &readResponse);
  if (readResponse & 0x02) {
    Serial.println(SER_GREEN "BUS_IORDY_REG (Ready indication) returned OK" SER_RESET);
  } else {
    Serial.printf(SER_RED "BUS_IORDY_REG (Ready indication) returned %d\n" SER_RESET, readResponse);
    return false;
  }

  ////////////////////////////////////
  //Included from SdFat init post CMD7
  ////////////////////////////////////

  //Set SDHC FIFO read/write water mark levels
  m_psdhc->WTMK_LVL = SDHC_WML_RDWML(FIFO_WML) | SDHC_WML_WRWML(FIFO_WML);

  //Set SDHC bus to 4-bits
  m_psdhc->PROT_CTRL &= ~SDHC_PROCTL_DTW_MASK;
  m_psdhc->PROT_CTRL |= SDHC_PROCTL_DTW(SDHC_PROCTL_DTW_4BIT);

  //Set the SDHC SCK frequency
  kHzSdClk = 25'000; //TODO 50'000

  //Disable GPIO
  enableSDIO(false);
  //Set clock
  setSdclk(kHzSdClk);
  //Enable GPIO
  enableSDIO(true);

  Serial.printf(SER_TRACE "\nSDHC bus set to 4-bit, speed set to %ldMHz\n" SER_RESET, kHzSdClk / 1000);

  ////////////////////////////////////////
  //End Included from SdFat init post CMD7
  ////////////////////////////////////////

  //Set backplane window
  setBackplaneWindow(BAK_BASE_ADDR);

  //Read chip ID 
  //This was 4 byte read in the Zero code, causes extra bytes in the buffer - only if first CMD53 executed. 
  //Changing size to < 4 "fixes" it. Debug code sdio.c 3909 formats value as %4x, so use size 2
  cardCMD53_read(SD_FUNC_BAK, SB_32BIT_WIN, u32d.bytes, 2);
  Serial.printf(SER_GREEN "\n*************\nCardID: %ld\n*************\n" SER_RESET, u32d.uint32 & 0xFFFF);

  ////////////////////////
  //Set chip clock - Zero
  ////////////////////////
  //sdio.c line 3913
  //Force PLL off until brcmf_chip_attach() programs PLL control regs
  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ); // Was 0x28

  //Validate ALP is available
  cardCMD52_read(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, &readResponse);
 if ((readResponse & ~(0x80 | 0x40)) == (SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ)) {
    Serial.printf(SER_GREEN "BAK_CHIP_CLOCK_CSR_REG returned 0x%02X\n" SER_RESET, readResponse);
  } else {
    Serial.printf(SER_RED "BAK_CHIP_CLOCK_CSR_REG returned 0x%02X\n" SER_RESET, readResponse);
    return false;
  }

  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_FORCE_ALP);  // Was 0x21
  ////////////////////////
  ////////////////////////

  //Disable pullups 
  cardCMD52_write(SD_FUNC_BAK, BAK_PULLUP_REG, 0);
  
  //Get chip ID again, and config base addr 
  cardCMD53_read(SD_FUNC_BAK, SB_32BIT_WIN, u32d.bytes, 4); // TODO Unused
  cardCMD53_read(SD_FUNC_BAK, SB_32BIT_WIN + 0xFC, u32d.bytes, 4); // TODO Unused

  //Start of download firmware
  
  //Reset cores
  backplaneWindow_write32(ARM_IOCTRL_REG, 0x03);
  backplaneWindow_write32(MAC_IOCTRL_REG, 0x07);
  backplaneWindow_write32(MAC_RESETCTRL_REG, 0x00);
  backplaneWindow_write32(MAC_IOCTRL_REG, 0x05);
  
  //[18.032572]
  backplaneWindow_write32(SRAM_IOCTRL_REG, 0x03);
  backplaneWindow_write32(SRAM_RESETCTRL_REG, 0x00);
  backplaneWindow_write32(SRAM_IOCTRL_REG, 0x01);
  
  if (!backplaneWindow_read32(SRAM_IOCTRL_REG, &u32d.uint32) || u32d.uint8 != 1) {
    Serial.println(SER_RED "Set SRAM_IOCTRL_REG issue" SER_RESET);
    return false;
  } else {
    Serial.println(SER_GREEN "Set SRAM_IOCTRL_REG validated" SER_RESET);
  }
  
  //[18.034039]
  //This is 4343x specific stuff: Disable remap for SRAM_3
  backplaneWindow_write32(SRAM_BANKX_IDX_REG, 0x03);
  backplaneWindow_write32(SRAM_BANKX_PDA_REG, 0x00);
  
  //[18.034733]
  if (!backplaneWindow_read32(SRAM_IOCTRL_REG, &u32d.uint32) || u32d.uint8 != 1) {
    Serial.println(SER_RED "Set SRAM_IOCTRL_REG issue" SER_RESET);
    return false;
  } else {
    Serial.println(SER_GREEN "Set SRAM_IOCTRL_REG validated" SER_RESET);
  }

  if (!backplaneWindow_read32(SRAM_RESETCTRL_REG, &u32d.uint32) || u32d.uint8 != 0) {
    Serial.println(SER_RED "Set SRAM_RESETCTRL_REG issue" SER_RESET);
    return false;
  } else {
    Serial.println(SER_GREEN "Set SRAM_RESETCTRL_REG validated" SER_RESET);
  }
  
  //[18.035416]
  backplaneWindow_read32(SRAM_BASE_ADDR, &u32d.uint32);
  backplaneWindow_write32(SRAM_BANKX_IDX_REG, 0);
  backplaneWindow_read32(SRAM_UNKNOWN_REG, &u32d.uint32);
  backplaneWindow_write32(SRAM_BANKX_IDX_REG, 1);
  backplaneWindow_read32(SRAM_UNKNOWN_REG, &u32d.uint32);
  backplaneWindow_write32(SRAM_BANKX_IDX_REG, 2);
  backplaneWindow_read32(SRAM_UNKNOWN_REG, &u32d.uint32);
  backplaneWindow_write32(SRAM_BANKX_IDX_REG, 3);
  
  //[18.037502]
  //Verify value at BUS_BRCM_CARDCTRL
  //sdio.c line 3991
  //Set card control so an SDIO card reset does a WLAN backplane reset
  cardCMD52_read(SD_FUNC_BUS, BUS_BRCM_CARDCTRL, &readResponse);
  if (readResponse == 1) {
    Serial.printf(SER_GREEN "BUS_BRCM_CARDCTRL returned %d\n" SER_RESET, readResponse);
  } else {
    Serial.printf(SER_RED "BUS_BRCM_CARDCTRL returned %d\n" SER_RESET, readResponse);
    return false;
  }
  readResponse |= SDIO_CCCR_BRCM_CARDCTRL_WLANRESET;
  cardCMD52_write(SD_FUNC_BUS, BUS_BRCM_CARDCTRL, readResponse);  // Was 3

  //sdio.c line 4002
  //Set PMUControl so a backplane reset does PMU state reload
  cardCMD53_read(SD_FUNC_BAK, 0x8600, u32d.bytes, 4);
  u32d.bytes[1] |= 0x40;
  cardCMD53_write(SD_FUNC_BAK, 0x8600, u32d.bytes, 4);
  
  // [18.052762]
  if (SDIOEnableFunction(SD_FUNC_BAK_EN) == false) {
    return false;
  }

  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, 0);
  delay(45);
  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, SBSDIO_ALP_AVAIL_REQ);
  
  //Validate BAK_CHIP_CLOCK_CSR_REG
  cardCMD52_read(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, &readResponse);
  if (readResponse == 0x48) {
    Serial.printf(SER_GREEN "BAK_CHIP_CLOCK_CSR_REG returned 0x%02X\n" SER_RESET, readResponse);
  } else {
    Serial.printf(SER_RED "BAK_CHIP_CLOCK_CSR_REG returned 0x%02X\n" SER_RESET, readResponse);
    return false;
  }
  
  //Validate and upload firmware 
  setBackplaneWindow(0x58000);
  cardCMD53_read(SD_FUNC_BAK, 0xEE80, data, 4);

  uploadFirmware(FIRMWARE_LEN, (uintptr_t)firmware_bin);

  size_t wifi_nvram_len = sizeof(wifi_nvram) - 1;
  uploadNVRAM(wifi_nvram_len, (uintptr_t)wifi_nvram);

  //This prints the last 44 bytes of NVRAM upload. Comment out for now
  //cardCMD53_read(SD_FUNC_BAK, 0xFFD4, data, 44);
  
  //[19.146150]
  backplaneWindow_read32(SRAM_IOCTRL_REG, &u32d.uint32); 
  backplaneWindow_read32(SRAM_RESETCTRL_REG, &u32d.uint32);
  backplaneWindow_write32(SB_INT_STATUS_REG, 0xffffffff);

  //[19.147404]
  backplaneWindow_write32(ARM_IOCTRL_REG, 0x03);
  backplaneWindow_write32(ARM_RESETCTRL_REG, 0x00);
  backplaneWindow_write32(ARM_IOCTRL_REG, 0x01);
  backplaneWindow_read32(ARM_IOCTRL_REG, &u32d.uint32);

  setBackplaneWindow(BAK_BASE_ADDR);
  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, 0);

  //Request HT Avail
  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, SBSDIO_HT_AVAIL_REQ);
  delay(50);

  //Validate HT Avail
  bool res = cardCMD52_read(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, &readResponse);
  if ((res == false) || readResponse != 0xD0) {
    Serial.printf(SER_RED "Set BAK_CHIP_CLOCK_CSR_REG issue, response: 0x%02X\n" SER_RESET, readResponse);
    return false;
  } else {
    Serial.println(SER_GREEN "Set BAK_CHIP_CLOCK_CSR_REG validated" SER_RESET);
  }
 
  //[19.190728]
  cardCMD52_write(SD_FUNC_BAK, BAK_CHIP_CLOCK_CSR_REG, 0xD2);
  backplaneWindow_write32(SB_TO_SB_MBOX_DATA_REG, 0x40000);

  if (SDIOEnableFunction(SD_FUNC_BAK_EN | SD_FUNC_RAD_EN) == false) {
    return false;
  }

  cardCMD52_read(SD_FUNC_BUS, BUS_IORDY_REG, &readResponse);
  delay(100);
  if (!cardCMD52_read(SD_FUNC_BUS, BUS_IORDY_REG, &readResponse) || readResponse != 0x06) {
    Serial.printf(SER_RED "Set BUS_IORDY_REG issue, response: 0x%02X\n" SER_RESET, readResponse);
    return false;
  } else {
    Serial.println(SER_GREEN "Set BUS_IORDY_REG validated" SER_RESET);
  }

  backplaneWindow_write32(SB_INT_HOST_MASK_REG, 0x200000F0);
  backplaneWindow_read32(SR_CONTROL1, &u32d.uint32);

  //[19.282972]
  setBackplaneWindow(BAK_BASE_ADDR);
  cardCMD52_write(SD_FUNC_BAK, BAK_WAKEUP_REG, 2);
  cardCMD52_write(SD_FUNC_BUS, BUS_BRCM_CARDCAP, 6);

  configureOOBInterrupt();
  cardCMD52_read(SD_FUNC_BUS, BUS_INTPEND_REG, &readResponse);


  //[19.284023]
  backplaneWindow_read32(SB_INT_STATUS_REG, &u32d.uint32);
  backplaneWindow_write32(SB_INT_STATUS_REG, 0x200000c0);
  backplaneWindow_read32(SB_TO_HOST_MBOX_DATA_REG, &u32d.uint32);
  backplaneWindow_write32(SB_TO_SB_MBOX_REG, 0x02);
  backplaneWindow_read32(SR_CONTROL1, &u32d.uint32);

  //[19.285708]
  backplaneWindow_read32(0x68000 | 0x7ffc, &u32d.uint32);
  setBackplaneWindow(0x38000);

  //TODO is this necessary? Comment out for now
  //cardCMD53_read(SD_FUNC_BAK, 0x70d4, data, 64);

  //[19.286520]
  backplaneWindow_read32(SB_INT_STATUS_REG, &u32d.uint32);
  backplaneWindow_write32(SB_INT_STATUS_REG, 0x80);

  cardCMD53_read(SD_FUNC_RAD, SB_32BIT_WIN, data, 64, false);
  
  Serial.printf("\n==============================\nEnd W4343WCard::begin: %s\n==============================\n", fUseSDIO2 ? "SDIO2" : "SDIO");
 
  return true;
}

//------------------------------------------------------------------------------
uint8_t W4343WCard::errorCode() const {
  return m_errorCode;
}
//------------------------------------------------------------------------------
uint32_t W4343WCard::errorData() const {
  return m_irqstat;
}
//------------------------------------------------------------------------------
uint32_t W4343WCard::errorLine() const {
  return m_errorLine;
}
//------------------------------------------------------------------------------
uint32_t W4343WCard::kHzSdClk() {
  return m_sdClkKhz;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Display fields in structure Added 02-21-25.
// Fields in descriptor are num:id (little-endian) or num:id (big_endian)
//------------------------------------------------------------------------------
void W4343WCard::disp_fields(void *data, char *fields, int maxlen)
{
    char *strs=fields, delim=0;
    uint8_t *dp = (uint8_t *)data;
    int n, dlen;
    int val;
    
    while (*strs && dp-(uint8_t *)data<maxlen)
    {
        dlen = 0;
        while (*strs>='0' && *strs<='9')
            dlen = dlen*10 + *strs++ - '0';
        delim = *strs++;
        if (*strs > ' ')
        {
            while (*strs >= '0')
                Serial.printf("%c",*strs++);
            Serial.printf("%c",'=');
            if (dlen <= 4)
            {
                val = 0;
                for (n=0; n<dlen; n++)
                    val |= (uint32_t)(*dp++) << ((delim==':' ? n : dlen-n-1) * 8);
                Serial.printf("%02X ", val);
            }
            else
            {
                for (n=0; n<dlen; n++)
                    Serial.printf("%02X", *dp++);
                Serial.printf("%c",' ');
            }
        }
        else
            dp += dlen;
        while (*strs == ' ')
            strs++;
    }
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Display block of data added 02-21-25
//----------------------------------------------------------------------
void W4343WCard::disp_block(uint8_t *data, int len)
{
    int i=0, n;

    while (i < len)
    {
        if (i > 0)
            Serial.printf("\n");
        n = MIN(len-i, DISP_BLOCKLEN);
        disp_bytes(&data[i], n);
        i += n;
        fflush(stdout);
    }
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Dump data as byte values Added 02-21-25
//----------------------------------------------------------------------
void W4343WCard::disp_bytes(uint8_t *data, int len)
{
    while (len--)
       Serial.printf("%02x ", *data++);
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Return string corresponding to event status Added 02-21-25
//----------------------------------------------------------------------
const char *W4343WCard::ioctl_evt_status_str(int status)
{
    return(status>=0 && status<MAX_EVENT_STATUS ? event_status[status] : "?");
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Return string corresponding to event number, without "WLC_E_" prefix Added 02-21-25
//----------------------------------------------------------------------
const char *W4343WCard::ioctl_evt_str(int event)
{
    EVT_STR *evtp=current_evts;

    while (evtp && evtp->num>=0 && evtp->num!=event)
        evtp++;
    return(evtp && evtp->num>=0 && strlen(evtp->str)>6 ? &evtp->str[6] : "?");
}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Set 2 integers in IOCTL variable Added 02-21-25
//----------------------------------------------------------------------
int W4343WCard::ioctl_set_intx2(char *name, int wait_msec, int val1, int val2)
{
    int data[2] = {val1, val2};

    return(ioctl_cmd(WLC_SET_VAR, name, wait_msec, 1, data, 8));
}
//----------------------------------------------------------------------
