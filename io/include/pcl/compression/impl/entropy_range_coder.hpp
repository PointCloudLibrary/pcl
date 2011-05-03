/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * range coder based on Dmitry Subbotin's carry-less implementation (http://www.compression.ru/ds/)
 * Added optimized symbol lookup and fixed/static frequency tables
 *
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef __PCL_IO_RANGECODING__HPP
#define __PCL_IO_RANGECODING__HPP

#include "pcl/compression/entropy_range_coder.h"

#include <map>
#include <iostream>
#include <vector>
#include <string.h>
#include <algorithm>
#include <stdio.h>

namespace pcl
{

  using namespace std;

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  AdaptiveRangeCoder::encodeCharVectorToStream (const std::vector<char>& inputByteVector_arg,
                                                std::ostream& outputByteStream_arg)
  {
    DWord freq[257];
    unsigned char ch;
    unsigned int i, j, f;
    char out;

    // define limits
    const DWord top = (DWord)1 << 24;
    const DWord bottom = (DWord)1 << 16;
    const DWord maxRange = (DWord)1 << 16;

    DWord low, range;

    unsigned int readPos;
    unsigned int input_size;

    input_size = inputByteVector_arg.size ();

    // init output vector
    outputCharVector_.clear();
    outputCharVector_.reserve(sizeof(char) * input_size);

    readPos = 0;

    low = 0;
    range = (DWord)-1;

    // initialize cumulative frequency table
    for (i = 0; i < 257; i++)
      freq[i] = i;

    // scan input
    while (readPos < input_size)
    {

      // read byte
      ch = inputByteVector_arg[readPos++];

      // map range
      low += freq[ch] * (range /= freq[256]);
      range *= freq[ch + 1] - freq[ch];

      // check range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        out = low >> 24;
        range <<= 8;
        low <<= 8;
        outputCharVector_.push_back(out);
      }

      // update frequency table
      for (j = ch + 1; j < 257; j++)
        freq[j]++;

      // detect overflow
      if (freq[256] >= maxRange)
      {
        // rescale
        for (f = 1; f <= 256; f++)
        {
          freq[f] /= 2;
          if (freq[f] <= freq[f - 1])
            freq[f] = freq[f - 1] + 1;
        }
      }

    }

    // flush remaining data
    for (i = 0; i < 4; i++)
    {
      out = low >> 24;
      outputCharVector_.push_back(out);
      low <<= 8;
    }

    // write to stream
    outputByteStream_arg.write (&outputCharVector_[0], outputCharVector_.size());
    outputByteStream_arg.flush ();

  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  AdaptiveRangeCoder::decodeStreamToCharVector (std::istream& inputByteStream_arg,
                                                std::vector<char>& outputByteVector_arg)
  {
    unsigned char ch;
    DWord freq[257];
    unsigned int i, j, f;

    // define limits
    const DWord top = (DWord)1 << 24;
    const DWord bottom = (DWord)1 << 16;
    const DWord maxRange = (DWord)1 << 16;

    DWord low, range;
    DWord code;

    unsigned int outputBufPos;
    unsigned int output_size = outputByteVector_arg.size ();

    outputBufPos = 0;

    code = 0;
    low = 0;
    range = (DWord)-1;

    // init decoding
    for (i = 0; i < 4; i++)
    {
      inputByteStream_arg.read ((char*)&ch, sizeof(char));
      code = (code << 8) | ch;
    }

    // init cumulative frequency table
    for (i = 0; i <= 256; i++)
      freq[i] = i;

    // decoding loop
    for (i = 0; i < output_size; i++)
    {
      unsigned char symbol = 0;
      unsigned char sSize = 256 / 2;

      // map code to range
      DWord count = (code - low) / (range /= freq[256]);

      // find corresponding symbol
      while (sSize > 0)
      {
        if (freq[symbol + sSize] <= count)
        {
          symbol += sSize;
        }
        sSize /= 2;
      }

      // output symbol
      outputByteVector_arg[outputBufPos++] = symbol;

      // update range limits
      low += freq[symbol] * range;
      range *= freq[symbol + 1] - freq[symbol];

      // decode range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        inputByteStream_arg.read ((char*)&ch, sizeof(char));
        code = code << 8 | ch;
        range <<= 8;
        low <<= 8;
      }

      // update cumulative frequency table
      for (j = symbol + 1; j < 257; j++)
        freq[j]++;

      // detect overflow
      if (freq[256] >= maxRange)
      {
        // rescale
        for (f = 1; f <= 256; f++)
        {
          freq[f] /= 2;
          if (freq[f] <= freq[f - 1])
            freq[f] = freq[f - 1] + 1;
        }
      }
    }

  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  StaticRangeCoder::encodeIntVectorToStream (std::vector<unsigned int>& inputIntVector_arg,
                                             std::ostream& outputByteStream_arg)
  {

    unsigned int inputsymbol;
    unsigned int i, f;
    char out;

    unsigned long frequencyTableSize;
    unsigned char frequencyTableByteSize;

    // define numerical limits
    const unsigned long top = (unsigned long)1 << 56;
    const unsigned long bottom = (unsigned long)1 << 48;
    const unsigned long maxRange = (unsigned long)1 << 48;

    unsigned long input_size = inputIntVector_arg.size ();
    unsigned long low, range;

    unsigned int inputSymbol;

    unsigned int readPos;


    // init output vector
    outputCharVector_.clear();
    outputCharVector_.reserve(sizeof(char) * input_size);

    frequencyTableSize = 1;

    readPos = 0;

    // calculate frequency table
    cFreqTable_[0] = cFreqTable_[1] = 0;
    while (readPos < input_size)
    {
      inputSymbol = inputIntVector_arg[readPos++];
      if (inputSymbol + 1 >= frequencyTableSize)
      {
        // frequency table is to small -> adaptively extend it
        unsigned long oldfrequencyTableSize;
        oldfrequencyTableSize = frequencyTableSize;

        do
        {
          // increase frequency table size by factor 2
          frequencyTableSize <<= 1;
        } while (inputSymbol + 1 > frequencyTableSize);

        if (cFreqTable_.size () < frequencyTableSize + 1)
        {
          // resize frequency vector
          cFreqTable_.resize (frequencyTableSize + 1);
        }

        // init new frequency range with zero
        memset (&cFreqTable_[oldfrequencyTableSize + 1], 0,
                sizeof(unsigned long) * (frequencyTableSize - oldfrequencyTableSize));
      }
      cFreqTable_[inputSymbol + 1]++;
    }
    frequencyTableSize++;

    // convert to cumulative frequency table
    for (f = 1; f < frequencyTableSize; f++)
    {
      cFreqTable_[f] = cFreqTable_[f - 1] + cFreqTable_[f];
      if (cFreqTable_[f] <= cFreqTable_[f - 1])
        cFreqTable_[f] = cFreqTable_[f - 1] + 1;
    }

    // rescale if numerical limits are reached
    while (cFreqTable_[frequencyTableSize - 1] >= maxRange)
    {
      for (f = 1; f < cFreqTable_.size (); f++)
      {
        cFreqTable_[f] /= 2;
        ;
        if (cFreqTable_[f] <= cFreqTable_[f - 1])
          cFreqTable_[f] = cFreqTable_[f - 1] + 1;
      }
    }

    // calculate amount of bytes per frequeny table entry
    frequencyTableByteSize = (unsigned char)ceil (Log2 (cFreqTable_[frequencyTableSize - 1]) / 8.0);

    // write size of frequency table to output stream
    outputByteStream_arg.write ((const char *)&frequencyTableSize, sizeof(frequencyTableSize));
    outputByteStream_arg.write ((const char *)&frequencyTableByteSize, sizeof(frequencyTableByteSize));

    // write cumulative  frequency table to output stream
    for (f = 1; f < frequencyTableSize; f++)
    {
      outputByteStream_arg.write ((const char *)&cFreqTable_[f], frequencyTableByteSize);
    }

    readPos = 0;
    low = 0;
    range = (unsigned long)-1;

    // start encoding
    while (readPos < input_size)
    {

      // read symol
      inputsymbol = inputIntVector_arg[readPos++];

      // map to range
      low += cFreqTable_[inputsymbol] * (range /= cFreqTable_[frequencyTableSize - 1]);
      range *= cFreqTable_[inputsymbol + 1] - cFreqTable_[inputsymbol];

      // check range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        out = low >> 56;
        range <<= 8;
        low <<= 8;
        outputCharVector_.push_back(out);
      }

    }

    // flush remaining data
    for (i = 0; i < 8; i++)
    {
      out = low >> 56;
      outputCharVector_.push_back(out);
      low <<= 8;
    }

    // write encoded data to stream
    outputByteStream_arg.write (&outputCharVector_[0], outputCharVector_.size());
    outputByteStream_arg.flush ();

  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  StaticRangeCoder::decodeStreamToIntVector (std::istream& inputByteStream_arg,
                                             std::vector<unsigned int>& outputIntVector_arg)
  {
    unsigned char ch;
    unsigned int i, f;

    // define range limits
    const unsigned long top = (unsigned long)1 << 56;
    const unsigned long bottom = (unsigned long)1 << 48;

    unsigned long low, range;
    unsigned long code;

    unsigned int outputBufPos;
    unsigned long output_size;

    unsigned long frequencyTableSize;
    unsigned char frequencyTableByteSize;

    outputBufPos = 0;
    output_size = outputIntVector_arg.size ();

    // read size of cumulative frequency table from stream
    inputByteStream_arg.read ((char*)&frequencyTableSize, sizeof(frequencyTableSize));
    inputByteStream_arg.read ((char*)&frequencyTableByteSize, sizeof(frequencyTableByteSize));

    // check size of frequency table vector
    if (cFreqTable_.size () < frequencyTableSize)
    {
      cFreqTable_.resize (frequencyTableSize);
    }

    // init with zero
    memset (&cFreqTable_[0], 0, sizeof(unsigned long) * frequencyTableSize);

    // read cumulative frequency table
    for (f = 1; f < frequencyTableSize; f++)
    {
      inputByteStream_arg.read ((char *)&cFreqTable_[f], frequencyTableByteSize);
    }

    // initialize range & code
    code = 0;
    low = 0;
    range = (unsigned long)-1;

    // init code vector
    for (i = 0; i < 8; i++)
    {
      inputByteStream_arg.read ((char*)&ch, sizeof(char));
      code = (code << 8) | ch;
    }

    // decoding
    for (i = 0; i < output_size; i++)
    {
      unsigned long count = (code - low) / (range /= cFreqTable_[frequencyTableSize - 1]);

      // sybmol lookup in cumulative frequency table
      unsigned long symbol = 0;
      unsigned long sSize = (frequencyTableSize - 1) / 2;
      while (sSize > 0)
      {
        if (cFreqTable_[symbol + sSize] <= count)
        {
          symbol += sSize;
        }
        sSize /= 2;
      }

      // write symbol to output stream
      outputIntVector_arg[outputBufPos++] = symbol;

      // map to range
      low += cFreqTable_[symbol] * range;
      range *= cFreqTable_[symbol + 1] - cFreqTable_[symbol];

      // check range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        inputByteStream_arg.read ((char*)&ch, sizeof(char));
        code = code << 8 | ch;
        range <<= 8;
        low <<= 8;
      }

    }

  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  StaticRangeCoder::encodeCharVectorToStream (const std::vector<char>& inputByteVector_arg,
                                              std::ostream& outputByteStream_arg)
  {
    DWord freq[257];
    unsigned char ch;
    int i, f;
    char out;

    // define numerical limits
    const DWord top = (DWord)1 << 24;
    const DWord bottom = (DWord)1 << 16;
    const DWord maxRange = (DWord)1 << 16;

    DWord low, range;

    unsigned int input_size;
    input_size = inputByteVector_arg.size ();

    unsigned int readPos;

    // init output vector
    outputCharVector_.clear();
    outputCharVector_.reserve(sizeof(char) * input_size);

    unsigned long FreqHist[257];
    unsigned long freqMaxrange;

    // calculate frequency table
    freqMaxrange = input_size;
    memset (FreqHist, 0, sizeof(FreqHist));
    readPos = 0;
    while (readPos < input_size)
    {
      FreqHist[(unsigned char)inputByteVector_arg[readPos++] + 1]++;
    }

    // convert to cumulative frequency table
    freq[0] = 0;
    for (f = 1; f <= 256; f++)
    {
      freq[f] = freq[f - 1] + (DWord)FreqHist[f];
      if (freq[f] <= freq[f - 1])
        freq[f] = freq[f - 1] + 1;
    }

    // rescale if numerical limits are reached
    while (freq[256] >= maxRange)
    {
      for (f = 1; f <= 256; f++)
      {
        freq[f] /= 2;
        ;
        if (freq[f] <= freq[f - 1])
          freq[f] = freq[f - 1] + 1;
      }
    }

    // write cumulative  frequency table to output stream
    outputByteStream_arg.write ((const char *)&freq[0], sizeof(freq));

    readPos = 0;

    low = 0;
    range = (DWord)-1;

    // start encoding
    while (readPos < input_size)
    {

      // read symol
      ch = inputByteVector_arg[readPos++];

      // map to range
      low += freq[ch] * (range /= freq[256]);
      range *= freq[ch + 1] - freq[ch];

      // check range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        out = low >> 24;
        range <<= 8;
        low <<= 8;
        outputCharVector_.push_back(out);
      }

    }

    // flush remaining data
    for (i = 0; i < 4; i++)
    {
      out = low >> 24;
      outputCharVector_.push_back(out);
      low <<= 8;
    }

    // write encoded data to stream
    outputByteStream_arg.write (&outputCharVector_[0], outputCharVector_.size());
    outputByteStream_arg.flush ();

  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void
  StaticRangeCoder::decodeStreamToCharVector (std::istream& inputByteStream_arg,
                                              std::vector<char>& outputByteVector_arg)
  {
    unsigned char ch;
    DWord freq[257];
    unsigned int i;

    // define range limits
    const DWord top = (DWord)1 << 24;
    const DWord bottom = (DWord)1 << 16;

    DWord low, range;
    DWord code;

    unsigned int outputBufPos;
    unsigned int output_size;

    output_size = outputByteVector_arg.size ();

    outputBufPos = 0;

    // read cumulative frequency table
    inputByteStream_arg.read ((char*)&freq[0], sizeof(freq));

    code = 0;
    low = 0;
    range = (DWord)-1;

    // init code
    for (i = 0; i < 4; i++)
    {
      inputByteStream_arg.read ((char*)&ch, sizeof(char));
      code = (code << 8) | ch;
    }

    // decoding
    for (i = 0; i < output_size; i++)
    {
      // symbol lookup in cumulative frequency table
      unsigned char symbol = 0;
      unsigned char sSize = 256 / 2;

      DWord count = (code - low) / (range /= freq[256]);

      while (sSize > 0)
      {
        if (freq[symbol + sSize] <= count)
        {
          symbol += sSize;
        }
        sSize /= 2;
      }

      // write symbol to output stream
      outputByteVector_arg[outputBufPos++] = symbol;

      low += freq[symbol] * range;
      range *= freq[symbol + 1] - freq[symbol];

      // check range limits
      while ((low ^ (low + range)) < top || ((range < bottom) && ((range = -low & (bottom - 1)), 1)))
      {
        inputByteStream_arg.read ((char*)&ch, sizeof(char));
        code = code << 8 | ch;
        range <<= 8;
        low <<= 8;
      }

    }

  }

}

#endif

