uniform sampler2D depth_tex;
uniform sampler2D measurement_tex;

void main(void)
{ 
  float texel = texture2D(depth_tex, gl_TexCoord[0].st).x;
  float texel2 = texture2D(measurement_tex, gl_TexCoord[1].st).x;
  gl_FragColor = gl_Color * texel * texel2;
}
