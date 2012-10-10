// does a 4x4 matrix transform on position
//
//// constants:
//uniform mat4 mvp; // model-view-projection matrix.
//uniform vec3 eye_position; 
//uniform vec3 light_direction;
//
//// inputs:
//attribute vec3 input_position;
//attribute vec3 input_normal;
//
//// outputs:
//// gl_Position (implicit)
//varying vec4 varying_color;
//
//void main() {
//	gl_Position = mvp * vec4(
//		input_position.x,
//		input_position.y,
//		input_position.z,
//		1
//	);
//	//float d = dot(input_normal,light_direction);
//	varying_color = vec4(1.0, 1.0, 1.0, 1);
//}
uniform mat4 MVPMat;    // Model-View-Projection matrix
attribute vec4 vPosition;

attribute vec4 a_color;
varying vec4 v_color;

//for the texture
attribute vec2 a_texCoord; 
varying vec2 v_texCoord;

const float fx_d = 5.5879981950414015e+02;
const float fy_d = 5.5874227168094478e+02;
const float cx_d = 3.1844162327317980e+02;
const float cy_d = 2.4574257294583529e+02;

void main()
{
    //perform the transformation on GPU
    //float real_depth = *(point_vertices+2);
    //float x = real_depth * (i - cx_d) * fx_d_1;
    //float y = real_depth * (j - cy_d) * fy_d_1;
    //transform the position so it will remap back to the real-world coordinate...
//    float z = vPosition.z;
//    float x = z * (vPosition.x-cx_d) / fx_d;
//    float y = z * (vPosition.y-cy_d) / fy_d;
//    float w = 1.0;
//    gl_Position = MVPMat*vec4(x,y,z,w);
//    gl_PointSize = 1.0;//vPosition[2];
//    v_color = a_color;
    
    float z = vPosition.z;
    float x = vPosition.x;
    float y = vPosition.y;
    float w = 1.0;
    gl_Position = MVPMat*vec4(x,y,z,w);
    //gl_Position = vec4(x,y,z,w);
    //gl_PointSize = 2.0;//vPosition[2];
    //v_color = vec4(1.0, 1.0, 1.0, 0.5);
    v_color = a_color+vec4(0.0, 0.0, 0.0, -0.5); //make it transparent
    v_texCoord = a_texCoord; //crashing the program?
}
