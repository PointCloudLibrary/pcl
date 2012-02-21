#version 330

uniform sampler2D Texture0;

in vec2 TexCoord0;
layout(location = 0) out vec4 FragColor;

void main() 
{
  FragColor = texture2D(Texture0, TexCoord0.st);
}
