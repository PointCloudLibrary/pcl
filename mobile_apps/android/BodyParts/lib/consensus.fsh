// define NUM_TREES
// define NUM_LABELS

precision mediump float;

uniform sampler2D multi_labels;

uniform int image_width, image_height;

int maxElementNoTie(int elements[NUM_LABELS])
{
  int max_element = 0;
  int max = elements[max_element];

  for (int i = 1; i < NUM_LABELS; ++i)
  {
    int val = elements[i];
    if (max < val) { max_element = i; max = val; }
    else if (max == val) { max_element = -1; }
  }

  return max_element;
}

int float2byte(float x)
{ return int(x * 255. + 0.5); }

void main()
{
    vec2 image_unit = vec2(1.0) / vec2(image_width, image_height);
    vec2 coords = gl_FragCoord.xy * image_unit;

    int bins[NUM_LABELS];

    for (int i = 0; i < NUM_LABELS; ++i)
        bins[i] = 0;

    vec4 multi_label_float = texture2D(multi_labels, coords);
    ivec4 multi_label = ivec4(multi_label_float * 255. + 0.5);

    for (int ti = 0; ti < NUM_TREES; ++ti)
        for (int i = 0; i < NUM_LABELS; ++i)
            if (multi_label[ti] == i)
                ++bins[i];

    int max_element = 0;
    int max = bins[0];

    for (int i = 1; i < NUM_LABELS; ++i)
    {
      int val = bins[i];
      if (max < val) { max_element = i; max = val; }
      else if (max == val) { max_element = -1; }
    }

    //gl_FragData[0] = vec4(float(maxElementNoTie(bins)) / 255., 0, 0, 1);
    gl_FragData[0] = vec4(float(max_element) / 255., 0, 0, 1);
}
