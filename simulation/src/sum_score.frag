#version 330
in vec2 TexCoord0;

layout(location = 0) out vec4 FragColor;

uniform float step_x;
uniform float step_y;

uniform sampler2D ArraySampler;

void main() 
{
  FragColor = texture2D(ArraySampler, TexCoord0.st) +
              texture2D(ArraySampler, TexCoord0.st + vec2(step_x, 0.0)) +
              texture2D(ArraySampler, TexCoord0.st + vec2(0.0, step_y)) +
              texture2D(ArraySampler, TexCoord0.st + vec2(step_x, step_y));
}
