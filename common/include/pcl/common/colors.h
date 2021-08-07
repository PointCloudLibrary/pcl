/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
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

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include <type_traits>  // for is_floating_point
#include <array>        // for std::array especially in Clang Darwin and MSVC

namespace pcl
{

  PCL_EXPORTS RGB
  getRandomColor (double min = 0.2, double max = 2.8);

  enum ColorLUTName
  {
    /** Color lookup table consisting of 256 colors structured in a maximally
      * discontinuous manner. Generated using the method of Glasbey et al.
      * (see https://github.com/taketwo/glasbey) */
    LUT_GLASBEY,
    /** A perceptually uniform colormap created by Stéfan van der Walt and
      * Nathaniel Smith for the Python matplotlib library.
      * (see https://youtu.be/xAoljeRJ3lU for background and overview) */
    LUT_VIRIDIS,
  };

  template <ColorLUTName T>
  class ColorLUT
  {

    public:

      /** Get a color from the lookup table with a given id.
        *
        * The id should be less than the size of the LUT (see size()). */
      static RGB at (std::size_t color_id);

      /** Get the number of colors in the lookup table.
        *
        * Note: the number of colors is different from the number of elements
        * in the lookup table (each color is defined by three bytes). */
      static std::size_t size ();

      /** Get a raw pointer to the lookup table. */
      static const unsigned char* data ();

  };

  using GlasbeyLUT = ColorLUT<pcl::LUT_GLASBEY>;
  using ViridisLUT = ColorLUT<pcl::LUT_VIRIDIS>;

  /**
   * @brief Returns a Look-Up Table useful in converting RGB to sRGB
   * @tparam T floating point type with resultant value
   * @tparam bits depth of RGB
   * @return 1-D LUT for converting R, G or B into Rs, Gs or Bs
   * @remarks sRGB was proposed by Stokes et al. as a uniform default color
   * space for the internet
   * M. Stokes, M. Anderson, S. Chandrasekar, and R. Motta: A standard default colorspace for the internet - sRGB (Nov 1996)
   * IEC 61966-2.1 Default RGB Colour Space - sRGB (International Electrotechnical Commission, Geneva, Switzerland, 1999)
   * www.srgb.com, www.color.org/srgb.html
   */
  template <typename T, std::uint8_t bits = 8>
  PCL_EXPORTS inline std::array<T, 1 << bits>
  RGB2sRGB_LUT() noexcept
  {
    static_assert(std::is_floating_point<T>::value, "LUT value must be a floating point");

    constexpr const std::size_t size = 1 << bits;

    static const auto sRGB_LUT = [&]() {
      // MSVC wouldn't take `size` here instead of the expression
      std::array<T, 1 << bits> LUT;
      for (std::size_t i = 0; i < size; ++i) {
        T f = static_cast<T>(i) / static_cast<T>(size - 1);
        if (f > 0.04045) {
          // ((f + 0.055)/1.055)^2.4
          LUT[i] = static_cast<T>(
              std::pow((f + static_cast<T>(0.055)) / static_cast<T>(1.055),
                       static_cast<T>(2.4)));
        }
        else {
          // f / 12.92
          LUT[i] = f / static_cast<T>(12.92);
        }
      }
      return LUT;
    }();
    return sRGB_LUT;
  }

  /**
   * @brief Returns a Look-Up Table useful in converting scaled CIE XYZ into CIE L*a*b*
   * @details The function assumes that the XYZ values are
   *   * not normalized using reference illuminant
   *   * scaled such that reference illuminant has Xn = Yn = Zn = discretizations
   * @tparam T floating point type with resultant value
   * @tparam discretizations number of levels for the LUT
   * @return 1-D LUT with results of f(X/Xn)
   * @note This function doesn't convert from CIE XYZ to CIE L*a*b*. The actual conversion
   * is as follows:
   * L* = 116 * [f(Y/Yn) - 16/116]
   * a* = 500 * [f(X/Xn) - f(Y/Yn)]
   * b* = 200 * [f(Y/Yn) - f(Z/Zn)]
   * Where, Xn, Yn and Zn are values of the reference illuminant (at prescribed angle)
   *        f is appropriate function such that L* = 100, a* = b* = 0 for white color
   * Reference: Billmeyer and Saltzman’s Principles of Color Technology
   */
  template <typename T, std::size_t discretizations = 4000>
  PCL_EXPORTS inline const std::array<T, discretizations>&
  XYZ2LAB_LUT() noexcept
  {
    static_assert(std::is_floating_point<T>::value, "LUT value must be a floating point");

    static const auto f_LUT = [&]() {
      std::array<T, discretizations> LUT;
      for (std::size_t i = 0; i < discretizations; ++i) {
        T f = static_cast<T>(i) / static_cast<T>(discretizations);
        if (f > static_cast<T>(0.008856)) {
          // f^(1/3)
          LUT[i] = static_cast<T>(std::pow(f, (static_cast<T>(1) / static_cast<T>(3))));
        }
        else {
          // 7.87 * f + 16/116
          LUT[i] =
              static_cast<T>(7.87) * f + (static_cast<T>(16) / static_cast<T>(116));
        }
      }
      return LUT;
    }();
    return f_LUT;
  }
}  // namespace pcl
