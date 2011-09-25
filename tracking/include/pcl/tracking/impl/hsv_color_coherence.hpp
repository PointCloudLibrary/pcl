#ifndef PCL_TRACKING_IMPL_HSV_COLOR_COHERENCE_H_
#define PCL_TRACKING_IMPL_HSV_COLOR_COHERENCE_H_

#include <Eigen/Dense>

namespace pcl
{
  namespace tracking
  {
    // utility
    typedef union
    {
      struct /*anonymous*/
      {
        unsigned char Blue; // Blue channel
        unsigned char Green; // Green channel
        unsigned char Red; // Red channel
      };
      float float_value;
      long long_value;
    } RGBValue;

    void RGB2HSV(int r, int g, int b, float& fh, float& fs, float& fv)
    {
      // mostly copied from opencv-svn/modules/imgproc/src/color.cpp
      // revision is 4351
      //int bidx = blueIdx, scn = srccn;
      const int hsv_shift = 12;
        
      static const int div_table[] = {
        0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
        130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
        65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
        43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
        32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
        26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
        21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
        18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
        16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
        14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
        13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
        11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
        10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
        10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
        9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
        8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
        8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
        7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
        7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
        6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
        6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
        6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
        5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
        5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
        5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
        5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
        5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
        4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
        4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
        4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
        4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
        4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
      };
      //int hr = hrange, hscale = hr == 180 ? 15 : 21;
      int hr = 180, hscale = 15;
      int h, s, v = b;
      int vmin = b, diff;
      int vr, vg;
                    
      v = std::max<int>(v, g);
      v = std::max<int>(v, r);
      vmin = std::min<int>(vmin, g);
      vmin = std::min<int>(vmin, r);
                
      diff = v - vmin;
      vr = v == r ? -1 : 0;
      vg = v == g ? -1 : 0;
                    
      s = diff * div_table[v] >> hsv_shift;
      h = (vr & (g - b)) +
        (~vr & ((vg & (b - r + 2 * diff))
                + ((~vg) & (r - g + 4 * diff))));
      h = (h * div_table[diff] * hscale +
           (1 << (hsv_shift + 6))) >> (7 + hsv_shift);
                
      h += h < 0 ? hr : 0;
      fh = h / 180.0;
      fs = s / 255.0;
      fv = v / 255.0;
    }
    
    template <typename PointInT> double
    HSVColorCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
    {
      // convert color space from RGB to HSV
      RGBValue source_rgb, target_rgb;
      source_rgb.float_value = (source.rgb);
      target_rgb.float_value = (target.rgb);

      float source_h, source_s, source_v, target_h, target_s, target_v;
      RGB2HSV (source_rgb.Red, source_rgb.Blue, source_rgb.Green,
               source_h, source_s, source_v);
      RGB2HSV (target_rgb.Red, target_rgb.Blue, target_rgb.Green,
               target_h, target_s, target_v);
      // hue value is in 0 ~ 2pi, but circulated.
      const float _h_diff = fabs (source_h - target_h);
      float h_diff;
      if ( _h_diff > 180.0 / 180.0 * M_PI )
      {
        h_diff = h_weight_ * (_h_diff - M_PI) * (_h_diff - M_PI) / M_PI / M_PI;
      }
      else
      {
        h_diff = h_weight_ * _h_diff * _h_diff / M_PI / M_PI;
      }

      const float s_diff = s_weight_ * (source_s - target_s) * (source_s - target_s);
      const float v_diff = v_weight_ * (source_v - target_v) * (source_v - target_v);
      const float diff2 = h_diff + s_diff + v_diff;
      
      return 1.0 / (1.0 + weight_ * diff2);
    }
  }
}

#define PCL_INSTANTIATE_HSVColorCoherence(T) template class PCL_EXPORTS pcl::tracking::HSVColorCoherence<T>;

#endif
