precision mediump float;
varying vec2 v_texCoord;  
uniform sampler2D s_texture_rgb;

void main()
{   
    //hack to give direct output without any processing
    //gl_FragColor=texture2D(s_texture_rgb, v_texCoord);
    gl_FragColor=texture2D(s_texture_rgb, v_texCoord)+vec4(0,0,0,1.0);
}