// inputs:
//precision mediump float;
//precision highp float;
//precision lowp float;
//varying vec4 varying_color;

// outputs:
// gl_FragColor (implicit)

//void main() {
	//direct pipe out
//    gl_FragColor = varying_color;
//}

precision mediump float;
varying vec2 v_texCoord;
uniform sampler2D s_texture_rgb;
varying vec4 v_color;
varying vec4 v_color2;
varying vec4 v_color3;

void main()
{    
//    //do lots of crazy things here
//    float count=0.0;
//    
//    vec3 j;
//    vec4 k=v_color;    
//    while(count<100.0){
//        k+=vec4(0.0001,0.0001,0.0001,0.0001);
//        vec3 a = vec3(1.0,0.0,4.0);
//        vec3 b = vec3(0.0,2.0,1.0);
//        float c=dot(a,b);
//        j = cross(a,b);
//        count = count+1.0;
//    }
//    k.w = 1.0;
    //gl_FragColor = texture2D(s_texture_rgb, v_texCoord);
    //gl_FragColor = vec4(0.0, 1.0, 0.0, 0.8);
//   gl_FragColor = k;
	gl_FragColor=texture2D(s_texture_rgb, v_texCoord);
	//-vec4(0,0,0,0.5);
    gl_FragColor = v_color;
}