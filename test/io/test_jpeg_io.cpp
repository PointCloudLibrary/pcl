/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Centrum Wiskunde Informatica.
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
 */
#include <gtest/gtest.h>
#include <pcl/common/io.h>
#include <pcl/PCLImage.h>
#include <pcl/io/jpeg_io.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <vector>

using namespace std;

// directory of jpeg should be given through argument on command line
string jpeg_dir;

// for storing filenames and loaded images
vector<pcl::PCLImage> ims_;
vector<string> jpeg_files_;

typedef vector<pcl::PCLImage>::iterator img_it_t;
typedef vector<string>::iterator str_it_t;

TEST (PCL, JPEGReadAndWrite)
{
  // load all the jpeg files
  img_it_t im_it = ims_.begin();
  std::cout << " number of jpeg files " << jpeg_files_.size() << std::endl;

  for(str_it_t it = jpeg_files_.begin(); it != jpeg_files_.end(); ++it, ++im_it)
  {
    // load the image file
    pcl::io::JPEGReader::readJPEG(*it, *im_it);
	
    EXPECT_GT(im_it->width , 0) << "incorrect image width given by jpegReader from file";
    EXPECT_GT(im_it->width , 0) << "incorrect image heigth given by jpegReader from file";

    // write to a compressed buffer
    std::vector<uint8_t> cdat;
    pcl::io::JPEGWriter::writeJPEG(*im_it,cdat);

    EXPECT_GT(cdat.size() , 0) << " no compressed data written by JpegWriter to Buffer";

    // read from the compressed buffer
    pcl::PCLImage im_out;
    pcl::io::JPEGReader::readJPEG(cdat, im_out);
    
    EXPECT_GT(im_out.width , 0) << "incorrect image width jpegReader from buffer";
    EXPECT_GT(im_out.height , 0) << "incorrect image heigth jpegReader from buffer";
    
    // check the image width and height
    EXPECT_EQ(im_out.width,im_it->width) << " in and output not equal in width ";
    EXPECT_EQ(im_out.height,im_it->height) << " in and output not equal in width ";
      
    // write the output decoded file to the folder with jpeg files
    using boost::filesystem::path;
    std::string out_name = (path(*it).parent_path() /= path("decoded") /= path("decoded_" + path(*it).filename().string())).string(); 
    bool res = pcl::io::JPEGWriter::writeJPEG(im_out, out_name );
    
    EXPECT_EQ((int)res,1) << " writing JPEG to file (JPEG Writer) failed";
  }
}

TEST (PCL, JPEGReaderIncorrectArguments)
{
   // opening an incorrect file
   pcl::PCLImage test_im;
   bool res1 = pcl::io::JPEGReader::readJPEG("randomincorrect_file.jpg",test_im);
   EXPECT_EQ((int) res1,0) << " jpeg reader did not return false for an incorrect file name ";

   // read jpeg from empty filename
   pcl::PCLImage test_im2;
   std::string empty_string;
   bool res2 = pcl::io::JPEGReader::readJPEG(empty_string,test_im2);
   EXPECT_EQ((int) res2,0) << " jpeg reader did not return false for an empty file name ";

   // read jpeg from incorrect buffer data
   std::vector<uint8_t> random_chars;
   pcl::PCLImage test_im3;
   for(int l=0;l< 89; l++)
   {
     random_chars.push_back(l);
   }
   bool res3 = pcl::io::JPEGReader::readJPEG(empty_string,test_im3);
   EXPECT_EQ((int) res3,0) << " jpeg reader did not return false for a bogus input buffer ";
}

TEST (PCL, JPEGWriterIncorrectArguments)
{
   // opening an incorrect file
   pcl::PCLImage test_im1;
   bool res1 = pcl::io::JPEGWriter::writeJPEG(test_im1, "some_out.jpg");
   EXPECT_EQ((int) res1,0) << " jpeg writer did not return false for an incorrect input image (empy one) ";

   // empty filename, valid image
  
   pcl::PCLImage test_im2;
   test_im2.width = 16;
   test_im2.height = 16;
   test_im2.data = std::vector<uint8_t> (16 * 16 * 3);
   test_im2.encoding = "RGB";
   test_im2.step = 3;
   
   std::string empty_string;
   bool res2 = pcl::io::JPEGWriter::writeJPEG(test_im2, empty_string);
   EXPECT_EQ((int) res2,0) << " jpeg writer did not return false for an incorrect file name (empty one) ";
}

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No folder with test files was given. Please add the path to jpeg_sequences to this test. (see pcl\\test\\jpeg_sequences) " << std::endl;
    return (-1);
  }

  std::string jpeg_sequences = argv[1];
  jpeg_dir = jpeg_sequences;
  
  // Get jpeg files
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (jpeg_dir); itr != end_itr; ++itr)
  {
#if BOOST_FILESYSTEM_VERSION == 3
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".JPG" )
#else
    if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->leaf ())) == ".JPG" )
#endif
    {
#if BOOST_FILESYSTEM_VERSION == 3
      jpeg_files_.push_back (itr->path ().string ());
      std::cout << "added: " << itr->path ().string () << std::endl;
#else
      jpeg_files_.push_back (itr->path().string ());
      std::cout << "added: " << itr->path() << std::endl;
#endif
    }
  }
  sort (jpeg_files_.begin (), jpeg_files_.end ());
  // And load them
  for (size_t i = 0; i < jpeg_files_.size (); i++)
  {
    pcl::PCLImage im;
    ims_.push_back (im);
  }

  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
