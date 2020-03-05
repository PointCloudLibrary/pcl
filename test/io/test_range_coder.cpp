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
 *   * Neither the name of the copyright holder(s) nor the names of its
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


#include <pcl/compression/entropy_range_coder.h>
#include <pcl/compression/impl/entropy_range_coder.hpp>

#include <pcl/test/gtest.h>
#include <vector>
#include <sstream>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Adaptive_Range_Coder_Test)
{
  std::stringstream sstream;
  std::vector<char> inputData;
  std::vector<char> outputData;

  unsigned long writeByteLen;
  unsigned long readByteLen;

  // vector size
  const unsigned int vectorSize = 10000;

  inputData.resize(vectorSize);
  outputData.resize(vectorSize);

  // fill vector with random data
  for (std::size_t i=0; i<vectorSize; i++)
  {
    inputData[i] = static_cast<char>(rand() & 0xFF);
  }

  // initialize adaptive range coder
  pcl::AdaptiveRangeCoder rangeCoder;

  // encode char vector to stringstream
  writeByteLen = rangeCoder.encodeCharVectorToStream(inputData, sstream);

  // decode stringstream to char vector
  readByteLen = rangeCoder.decodeStreamToCharVector(sstream, outputData);

  // compare amount of bytes that are read and written to/from stream
  EXPECT_EQ (writeByteLen, readByteLen);
  EXPECT_EQ (writeByteLen, sstream.str().length());

  // compare input and output vector - should be identical
  EXPECT_EQ (inputData.size(), outputData.size());
  EXPECT_EQ (inputData.size(), vectorSize);

  for (std::size_t i=0; i<vectorSize; i++)
  {
    EXPECT_EQ (inputData[i], outputData[i]);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, Static_Range_Coder_Test)
{
  // Run test for different vector sizes
  for (unsigned int vectorSize: { 253, 10000 })
  {
    std::stringstream sstream;
    std::vector<char> inputCharData;
    std::vector<char> outputCharData;

    std::vector<unsigned int> inputIntData;
    std::vector<unsigned int> outputIntData;

    unsigned long writeByteLen;
    unsigned long readByteLen;

    inputCharData.resize(vectorSize);
    outputCharData.resize(vectorSize);

    inputIntData.resize(vectorSize);
    outputIntData.resize(vectorSize);

    // fill vectors with random data
    for (std::size_t i=0; i<vectorSize; i++)
    {
      inputCharData[i] = static_cast<char> (rand () & 0xFF);
      inputIntData[i] = static_cast<unsigned int> (rand () & 0xFFFF);
    }

    // initialize static range coder
    pcl::StaticRangeCoder rangeCoder;

    // encode char vector to stringstream
    writeByteLen = rangeCoder.encodeCharVectorToStream(inputCharData, sstream);

    // decode stringstream to char vector
    readByteLen = rangeCoder.decodeStreamToCharVector(sstream, outputCharData);

    // compare amount of bytes that are read and written to/from stream
    EXPECT_EQ (writeByteLen, readByteLen);
    EXPECT_EQ (writeByteLen, sstream.str().length());

    // compare input and output vector - should be identical
    EXPECT_EQ (inputCharData.size(), outputCharData.size());
    EXPECT_EQ (inputCharData.size(), vectorSize);

    for (std::size_t i=0; i<vectorSize; i++)
    {
      EXPECT_EQ (inputCharData[i], outputCharData[i]);
    }

    sstream.clear();

    // encode integer vector to stringstream
    writeByteLen = rangeCoder.encodeIntVectorToStream(inputIntData, sstream);

    // decode stringstream to integer vector
    readByteLen = rangeCoder.decodeStreamToIntVector(sstream, outputIntData);

    // compare amount of bytes that are read and written to/from stream
    EXPECT_EQ (writeByteLen, readByteLen);

    // compare input and output vector - should be identical
    EXPECT_EQ (inputIntData.size(), outputIntData.size());
    EXPECT_EQ (inputIntData.size(), vectorSize);

    for (std::size_t i=0; i<vectorSize; i++)
    {
      EXPECT_EQ (inputIntData[i], outputIntData[i]);
    }
  }
}



/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
