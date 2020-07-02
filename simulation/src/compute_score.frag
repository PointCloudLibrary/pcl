#version 330
in vec2 TexCoord0;

layout(location = 0) out vec4 FragColor;

uniform sampler2D DepthSampler;
uniform sampler2D ReferenceSampler;
uniform sampler2D CostSampler;

uniform int cols;
uniform int rows;
uniform float near;
uniform float far;

void main() 
{
  float n = near;
  float f = far;
    
  float reference_depth_val = texture2D(ReferenceSampler, vec2(TexCoord0.s*cols, TexCoord0.t*rows)).r;
  float depth_val = texture2D(DepthSampler, TexCoord0.st).r; 
  float ref_meters = 1.0 / (reference_depth_val * (1.0/f - 1.0/n) + 1.0/n);
  float depth_meters = 1.0 / (depth_val * (1.0/f - 1.0/n) + 1.0/n); 
  float min_dist = abs(ref_meters - depth_meters);
 
  float likelihood = texture2D(CostSampler, vec2(clamp(min_dist/3.0, 0.0, 1.0),0.0)).r;
  float ratio = 0.99;
  float r_min = 0.0; // meters
  float r_max = 3.0; // meters
  
  likelihood = log(ratio/(r_max - r_min) + (1.0 - ratio)*likelihood);
  if (reference_depth_val < 0.0) likelihood = 0.0;
  
  FragColor = vec4(likelihood, 0, 0, 0);
}
