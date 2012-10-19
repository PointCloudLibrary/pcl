// define TREE_DEPTH
// define NODES_HEIGHT

precision highp float;
precision highp int;

const int BACKGROUND_DEPTH = 32767;
const float BACKGROUND_LABEL = 29./255.;
const float TREE_WIDTH = 2048.;
const float TREE_RWIDTH = 1.0 / TREE_WIDTH;

uniform sampler2D depth_image;
uniform sampler2D offsets1, offsets2, thresholds;
uniform sampler2D leaves;

uniform int image_width, image_height;

int xy2int(vec2 v)
{
    int left = int(v.x * 255. + 0.5), right = int(v.y * 255. + 0.5);
    if (right <= 127) return left + right * 256; else return -(255 - left + (255 - right) * 256) - 1;
}

void main()
{
  vec2 image_unit = vec2(1.0) / vec2(image_width, image_height);
  vec2 tree_unit = vec2(TREE_RWIDTH);
  vec2 coords = gl_FragCoord.xy * image_unit;

  vec2 tree_pos = vec2(0, 0);

  vec4 d0_vec = texture2D(depth_image, coords);
  int d0 = xy2int(d0_vec.xy);

  if (d0 >= 32766)
  {
    gl_FragData[0] = vec4(BACKGROUND_LABEL, 0, 0, 1);
    return;
  }

  float scale = 1000. / float(d0);

  for (int i = 0; i < TREE_DEPTH; ++i)
  {
    vec4 offset1_vec = texture2D(offsets1, tree_pos * tree_unit);
    vec2 offset1 = vec2(xy2int(offset1_vec.xy), xy2int(offset1_vec.zw));
    vec4 d1_vec = texture2D(depth_image, coords + offset1 * scale * image_unit);
    int d1 = xy2int(d1_vec.xy);

    vec4 offset2_vec = texture2D(offsets2, tree_pos * tree_unit);
    vec2 offset2 = vec2(xy2int(offset2_vec.xy), xy2int(offset2_vec.zw));
    vec4 d2_vec = texture2D(depth_image, coords + offset2 * scale * image_unit);
    int d2 = xy2int(d2_vec.xy);

    vec4 threshold_vec = texture2D(thresholds, tree_pos * tree_unit);
    int threshold = xy2int(threshold_vec.xy);

    tree_pos.y *= 2.;
    tree_pos.x = 2. * tree_pos.x + 1. + float(d1 - d2 > threshold);
    if (tree_pos.x >= 2. * TREE_WIDTH)
    {
        tree_pos.y += 2.;
        tree_pos.x -= 2. * TREE_WIDTH;
    }
    else if (tree_pos.x >= TREE_WIDTH)
    {
        tree_pos.y += 1.;
        tree_pos.x -= TREE_WIDTH;
    }
  }

  if (tree_pos.x + 1. == TREE_WIDTH)
  {
      tree_pos.x == 0.;
      tree_pos.y += 1.;
  }
  else
  {
      tree_pos.x += 1.;
  }

  tree_pos.y -= float(NODES_HEIGHT);

  gl_FragData[0] = texture2D(leaves, tree_pos * tree_unit);
}
