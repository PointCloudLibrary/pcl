#version 330

layout (location = 0) in vec3 Position;
layout (location = 1) in vec2 TexCoord;

out vec2 TexCoord0;

void main()
{
  TexCoord0 = TexCoord;
  gl_Position = vec4(Position, 1.0);
};
