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
#ifndef SdCardInterface_h
#define SdCardInterface_h
#include "SdCardInfo.h"
/**
 * \class SdCardInterface
 * \brief Abstract interface for an SD card.
 */
class SdCardInterface { // : public FsBlockDeviceInterface {
 public:

  /** \return error code. */
  virtual uint8_t errorCode() const = 0;
  /** \return error data. */
  virtual uint32_t errorData() const = 0;
  /** \return card status. */
  virtual uint32_t status() {return 0XFFFFFFFF;}
  /** Return the card type: SD V1, SD V2 or SDHC/SDXC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC.
   */
  virtual uint8_t type() const = 0;
  /** Write one data sector in a multiple sector write sequence.
   * \param[in] src Pointer to the location of the data to be written.
   * \return true for success or false for failure.
   */

  virtual bool writeData(const uint8_t* src) = 0;
  /** Start a write multiple sectors sequence.
   *
   * \param[in] sector Address of first sector in sequence.
   *
   * \return true for success or false for failure.
   */
  virtual bool writeStart(uint32_t sector) = 0;
  /** End a write multiple sectors sequence.
   * \return true for success or false for failure.
   */
};
#endif  // SdCardInterface_h
