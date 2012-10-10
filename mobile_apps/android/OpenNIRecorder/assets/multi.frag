precision mediump float;
varying vec2 v_texCoord;  
varying vec2 v_texCoord_depth;  
varying float v_depth;
uniform sampler2D s_texture_rgb;
uniform sampler2D s_texture_depth;

//my own calibrated data.. replace the following with your own.

const float fx_rgb = 5.0995627581871054e+02;
const float fy_rgb = 5.1009672745532589e+02;
const float cx_rgb = 3.2034728823678967e+02;
const float cy_rgb = 2.5959956434787728e+02;
const float k1_rgb = 2.6172416643533958e-01;
const float k2_rgb = -8.2104703257074252e-01;
const float p1_rgb = -1.0637850248230928e-03;
const float p2_rgb = 8.4946289275097779e-04;
const float k3_rgb = 8.9012728224037985e-01; 

//DEPTH CAMERA
const float fx_d = 5.5879981950414015e+02;
const float fy_d = 5.5874227168094478e+02;
const float cx_d = 3.1844162327317980e+02;
const float cy_d = 2.4574257294583529e+02;
const float k1_d = -2.6167161458989197e-01;
const float k2_d = 9.9319844495479259e-01;
const float p1_d = -1.0221823575713733e-03;
const float p2_d = 5.3621541487535148e-03;
const float k3_d = -1.2966457585622253e+00;

//Size of the image, use to transform from texture coord to image coord..
vec2 img_coord = vec2(640.0,480.0);

//for fixing radial distortion
vec2 lookup(vec2 coord,
                 in float fx, 
                 in float fy,
                 in float cx,
                 in float cy,
                 in float k1, 
                 in float k2,
                 in float p1,
                 in float p2 ) 
{
   vec2 ff = vec2(fx,fy); 
   coord = (coord-vec2(cx,cy))/ff;
   float x1 = coord.x;
   float y1 = coord.y;
   
   float r = sqrt((x1*x1)+(y1*y1));
   float r2 = r*r;
   float r4 = r2*r2;
   float coeff = (1.0+k1*r2+k2*r4);

   float dx = 2.0*p1*x1*y1 + p2*(r2+2.0*x1*x1);
   float dy = p1*(r2+2.0*y1*y1) + 2.0*p2*x1*y1;

   coord = ((coord*vec2(coeff,coeff) + vec2(dx,dy))*ff)+vec2(cx,cy);
   return coord;
}

//octave:15> rM'*(T4)
//   0.99985  -0.00148   0.01747  -0.01979
//   0.00126   0.99992   0.01228   0.00085
//  -0.01749  -0.01225   0.99977   0.01125
//   0.00000   0.00000   0.00000   1.00000
//vec4 M1 = vec4(0.99985,  -0.00148,  0.01747,  -0.01979);
//vec4 M2 = vec4(0.00126, 0.99992,  0.01228,   0.00085);
//vec4 M3 = vec4(-0.01749, -0.01225,  0.99977,   0.01125);
//vec4 M4 = vec4(0.00000, 0.00000,   0.00000,  1.00000);  

//this matrix is the transpose of the above, but we have taken out the last row because
//it is not necessary here.
vec3 M1 = vec3(0.99977,   0.00173,  -0.02123);
vec3 M2 = vec3(-0.00200,   0.99991,  -0.01289);
vec3 M3 = vec3(0.02120,   0.01293,   0.99969);
vec3 M4 = vec3(-0.02107,  -0.00238,   0.01340);  

//Rotation and translation matrix
vec3 T = vec3(2.7127130138943419e-02, -1.0041314603411674e-03,-5.6746227781378283e-03);
vec3 R1 = vec3(9.9996078957902945e-01, -8.5633968850082568e-03,-2.2555571980713987e-03);
vec3 R2 = vec3(8.5885385454046812e-03, 9.9989832404109968e-01, 1.1383258999693677e-02);
vec3 R3 = vec3(2.1578484974712269e-03, -1.1402184597253283e-02, 9.9993266467111286e-01);
//vec3 R1 = vec3(9.9977321644139494e-01, -2.0032487074002391e-03, 2.1201478274968936e-02);
//vec3 R2 = vec3(1.7292658422779497e-03, 9.9991486643051353e-01, 1.2933272242365573e-02);
//vec3 R3 = vec3(-2.1225581878346968e-02, -1.2893676196675344e-02, 9.9969156632836553e-01);
  
float offset_x = 1.0/640.0;
float offset_y = 1.0/480.0;
vec3 TransformPoint(vec3 point)
{
//T=[1,0,0,-2.1354778990792557e-02; 0,1,0,-2.5073334719943473e-03; 0, 0, 1, 1.2922411623995907e-02; 0, 0, 0, 1];
//R =[9.9977321644139494e-01, 1.7292658422779497e-03, -2.1225581878346968e-02, 0; -2.0032487074002391e-03, 9.9991486643051353e-01, -1.2893676196675344e-02, 0; 2.1201478274968936e-02, 1.2933272242365573e-02, 9.9969156632836553e-01, 0; 0, 0, 0, 1];
 
//octave:9> (rM*T)'
//   0.99977   0.00173  -0.02123   0.00000
//  -0.00200   0.99991  -0.01289   0.00000
//   0.02120   0.01293   0.99969   0.00000
//  -0.02107  -0.00238   0.01340   1.00000

	//hacked version for speed.
   	float w = dot(point, M4) + 1.0;
   	if(w!=0.0)
   	{
   		float invW = 1.0 / w;
        return vec3( dot(point, M1) * invW,
                     dot(point, M2)  * invW,
                     dot(point, M3)  * invW);
   	}else{
   		return vec3(0,0,0);
   	}
}

void main()
{   
    //gl_FragColor = vec4(1.0,0.0,0.0,1.0);
    //return;
    //direct output
    //gl_FragColor=texture2D(s_texture_rgb, v_texCoord_depth);
    //return;
	//use this to fix radial distortion, not working yet
	//vec2 depth_loc = lookup(v_texCoord*vec2(640.0,480.0), fx_d, fy_d, cx_d, cy_d, k1_d, k2_d, p1_d, p2_d)/vec2(640.0,480.0);
    //float depth = v_depth;
    //do an average depth!
    
//    float depth1 = texture2D(s_texture_depth, v_texCoord_depth.x+vec2(-offset_x,-offset_y)).w;	
//	float depth2 = texture2D(s_texture_depth, v_texCoord_depth+vec2(offset_x,-offset_y)).w;	
//	float depth3 = texture2D(s_texture_depth, v_texCoord_depth+vec2(-offset_x,offset_y)).w;	
//	float depth4 = texture2D(s_texture_depth, v_texCoord_depth+vec2(offset_x,offset_y)).w;	
//	float depth5 = texture2D(s_texture_depth, v_texCoord_depth+vec2(0,0)).w;	
//	float depth6 = texture2D(s_texture_depth, v_texCoord_depth+vec2(0,offset_y)).w;	
//	float depth7 = texture2D(s_texture_depth, v_texCoord_depth+vec2(offset_x,0)).w;	
//	float depth8 = texture2D(s_texture_depth, v_texCoord_depth+vec2(-offset_x,0)).w;	
//	float depth9 = texture2D(s_texture_depth, v_texCoord_depth+vec2(0,-offset_y)).w;	
//	float depth = depth1+depth2+depth3+depth4+depth5+depth6+depth7+depth8+depth9;
//    depth=depth/9.0;
    
    float depth = texture2D(s_texture_depth, v_texCoord_depth).w;	
	if(depth == 0.0 || depth == 1.0){
		gl_FragColor = vec4(0, 0, 0, 1);
		return;
	}
	
    //transform to image coordinate first, texture coord is from 0 to 1
	float x_d = (v_texCoord_depth.x)*img_coord.x;
	float y_d = (v_texCoord_depth.y)*img_coord.y;
	
	//vec2 xy_d = v_texCoord*img_coord;
				
    vec3 P3D;
	vec3 P3D_1;
	vec2 P2D_rgb;
	float real_depth = (8.0 * depth)+0.35;
		
    //this should be in metric 3D space (world coordinate) ... to verify this? yes I did x]
    P3D.x = real_depth * (x_d - cx_d) / fx_d;
	P3D.y = -real_depth * (y_d - cy_d) / fy_d;
	P3D.z = real_depth;
	

	//transform this then project to the camera (extrinsic parameters etc...)
	//P3D_1  = TransformPoint(P3D); //slower but cleaner code
	float w = dot(P3D, M4) + 1.0;
	//float invW = 1.0 / w;
	//perform the operation using the pre-computed matrix
    //P3D_1=vec3( dot(P3D, M1) * invW, dot(P3D, M2) * invW, dot(P3D, M3) * invW);
    
	P3D_1 = vec3(dot(R1, P3D)-T.x, dot(R2, P3D)-T.y, dot(R3, P3D)-T.z);
   	//P3D_1 = P3D;
   	
	//now we map this back to the depth image.
	float P3D_1_1 = 1.0 / P3D_1.z;
	
	//float x1 = P3D_1.x*P3D_1_1;
	//float y1 = P3D_1.y*P3D_1_1;
	
	//float r = (x1*x1)+(y1*y1);
   	//float r2 = (x1*x1)+(y1*y1);
   	//float r4 = r2*r2;
   	//float coeff = (1.0+k1_rgb*r2+k2_rgb*r4);

   	//float dx = 2.0*p1_rgb*x1*y1 + p2_rgb*(r2+2.0*x1*x1);
   	//float dy = p1_rgb*(r2+2.0*y1*y1) + 2.0*p2_rgb*x1*y1;
   	//P3D_1.x = (x1*coeff)+dx;
    //P3D_1.y = (y1*coeff)+dy;
   	
	//location of the RGB pixels to be mapped back to depth.
	P2D_rgb.x = (P3D_1.x * fx_rgb * P3D_1_1) + cx_rgb;
	P2D_rgb.y = -(P3D_1.y * fy_rgb * P3D_1_1) + cy_rgb;
	
	//transform back to texture coord
	P2D_rgb = P2D_rgb/img_coord;
	
	//vec2 rgb_loc = lookup(P2D_rgb, fx_rgb, fy_rgb, cx_rgb, cy_rgb, k1_rgb, k2_rgb, p1_rgb, p2_rgb)/vec2(640.0,480.0);
	
	//store the final result
	gl_FragColor=vec4(1.0, 1.0, 1.0, 0)*texture2D(s_texture_rgb, P2D_rgb)+vec4(0,0,0,1.0);
	//gl_FragColor=vec4(1.0, 1.0, 1.0, 0)*texture2D(s_texture_rgb, P2D_rgb)+vec4(0.0,0.0,0.0,1.0);
}