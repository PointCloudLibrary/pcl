/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 ========================================================================*/
/// \file vesShader_vert.glsl
///
/// \ingroup shaders

// Uniforms.
uniform bool    hasVertexColors;
uniform mediump vec3 lightDirection;
uniform highp mat4   modelViewMatrix;
uniform mediump mat3 normalMatrix;
uniform lowp int     primitiveType;
uniform highp mat4 projectionMatrix;

// Vertex attributes.
attribute highp vec3   vertexPosition;
attribute mediump vec3 vertexNormal;
attribute lowp vec4    vertexColor;

// Varying attributes.
varying lowp vec4 varColor;

void main()
{
  // Save position for shading later.
  highp vec4 position = projectionMatrix * modelViewMatrix * vec4(vertexPosition, 1.0);

  varColor = vertexColor;

  // 1 is line
  if (primitiveType != 1 && primitiveType != 0) {
    // Transform vertex normal into eye space.
    lowp vec3 normal = normalize(normalMatrix * vertexNormal);

    // Save light direction (direction light for now)
    lowp vec3 lightDirection = normalize(vec3(0.0, 0.0, 0.650));

    lowp float nDotL = max(dot(normal, lightDirection), 0.0);

    // Do backface lighting too.
    nDotL = max(dot(-normal, lightDirection), nDotL);

    varColor = vec4(varColor.xyz * nDotL, varColor.w);
  }

  gl_PointSize = 1.0;
  gl_Position = position;
}
