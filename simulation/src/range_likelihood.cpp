#include <GL/glew.h>

#include <pcl/pcl_config.h>
#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif

#include <pcl/common/time.h>
#include <pcl/simulation/range_likelihood.h>

// For adding noise:
static boost::minstd_rand generator (27u); // seed

//#define SIMULATION_DEBUG 1
#define DO_TIMING_PROFILE 0

using namespace std;

// 301 values, 0.0 uniform  1.0 normal. properly truncated/normalized
float normal_sigma0x5_normal1x0_range0to3_step0x01[] = {1.59576912f, 1.59545000f, 1.59449302f, 1.59289932f, 1.59067083f,
                                                        1.58781019f, 1.58432085f, 1.58020697f, 1.57547345f, 1.57012594f,
                                                        1.56417078f, 1.55761504f, 1.55046646f, 1.54273348f, 1.53442517f,
                                                        1.52555126f, 1.51612211f, 1.50614865f, 1.49564242f, 1.48461552f,
                                                        1.47308056f, 1.46105069f, 1.44853953f, 1.43556117f, 1.42213012f,
                                                        1.40826131f, 1.39397005f, 1.37927201f, 1.36418316f, 1.34871978f,
                                                        1.33289841f, 1.31673585f, 1.30024906f, 1.28345522f, 1.26637163f,
                                                        1.24901574f, 1.23140504f, 1.21355714f, 1.19548963f, 1.17722012f,
                                                        1.15876621f, 1.14014544f, 1.12137525f, 1.10247299f, 1.08345589f,
                                                        1.06434100f, 1.04514521f, 1.02588518f, 1.00657737f, 0.98723796f,
                                                        0.96788290f, 0.94852781f, 0.92918802f, 0.90987853f, 0.89061400f,
                                                        0.87140871f, 0.85227659f, 0.83323116f, 0.81428555f, 0.79545248f,
                                                        0.77674422f, 0.75817263f, 0.73974913f, 0.72148465f, 0.70338972f,
                                                        0.68547437f, 0.66774817f, 0.65022022f, 0.63289916f, 0.61579315f,
                                                        0.59890986f, 0.58225652f, 0.56583986f, 0.54966616f, 0.53374122f,
                                                        0.51807038f, 0.50265855f, 0.48751015f, 0.47262918f, 0.45801920f,
                                                        0.44368334f, 0.42962430f, 0.41584438f, 0.40234547f, 0.38912908f,
                                                        0.37619631f, 0.36354792f, 0.35118428f, 0.33910545f, 0.32731110f,
                                                        0.31580063f, 0.30457310f, 0.29362725f, 0.28296157f, 0.27257426f,
                                                        0.26246326f, 0.25262625f, 0.24306068f, 0.23376378f, 0.22473257f,
                                                        0.21596387f, 0.20745431f, 0.19920035f, 0.19119830f, 0.18344431f,
                                                        0.17593438f, 0.16866443f, 0.16163022f, 0.15482742f, 0.14825164f,
                                                        0.14189837f, 0.13576305f, 0.12984106f, 0.12412773f, 0.11861834f,
                                                        0.11330815f, 0.10819240f, 0.10326630f, 0.09852508f, 0.09396394f,
                                                        0.08957812f, 0.08536286f, 0.08131342f, 0.07742511f, 0.07369324f,
                                                        0.07011320f, 0.06668040f, 0.06339032f, 0.06023847f, 0.05722044f,
                                                        0.05433188f, 0.05156850f, 0.04892611f, 0.04640054f, 0.04398775f,
                                                        0.04168374f, 0.03948462f, 0.03738655f, 0.03538582f, 0.03347876f,
                                                        0.03166181f, 0.02993149f, 0.02828442f, 0.02671730f, 0.02522691f,
                                                        0.02381013f, 0.02246393f, 0.02118538f, 0.01997160f, 0.01881983f,
                                                        0.01772739f, 0.01669169f, 0.01571021f, 0.01478053f, 0.01390031f,
                                                        0.01306728f, 0.01227925f, 0.01153414f, 0.01082990f, 0.01016460f,
                                                        0.00953635f, 0.00894336f, 0.00838388f, 0.00785626f, 0.00735890f,
                                                        0.00689028f, 0.00644891f, 0.00603340f, 0.00564241f, 0.00527464f,
                                                        0.00492888f, 0.00460393f, 0.00429869f, 0.00401209f, 0.00374309f,
                                                        0.00349073f, 0.00325408f, 0.00303227f, 0.00282444f, 0.00262981f,
                                                        0.00244761f, 0.00227712f, 0.00211766f, 0.00196858f, 0.00182926f,
                                                        0.00169912f, 0.00157761f, 0.00146420f, 0.00135840f, 0.00125975f,
                                                        0.00116779f, 0.00108211f, 0.00100231f, 0.00092803f, 0.00085891f,
                                                        0.00079462f, 0.00073485f, 0.00067930f, 0.00062770f, 0.00057979f,
                                                        0.00053532f, 0.00049406f, 0.00045581f, 0.00042034f, 0.00038748f,
                                                        0.00035705f, 0.00032887f, 0.00030280f, 0.00027868f, 0.00025638f,
                                                        0.00023577f, 0.00021673f, 0.00019915f, 0.00018292f, 0.00016795f,
                                                        0.00015414f, 0.00014141f, 0.00012968f, 0.00011887f, 0.00010893f,
                                                        0.00009977f, 0.00009135f, 0.00008360f, 0.00007648f, 0.00006994f,
                                                        0.00006393f, 0.00005842f, 0.00005336f, 0.00004872f, 0.00004446f,
                                                        0.00004056f, 0.00003699f, 0.00003372f, 0.00003072f, 0.00002798f,
                                                        0.00002548f, 0.00002319f, 0.00002110f, 0.00001918f, 0.00001744f,
                                                        0.00001585f, 0.00001439f, 0.00001307f, 0.00001186f, 0.00001076f,
                                                        0.00000976f, 0.00000884f, 0.00000801f, 0.00000726f, 0.00000657f,
                                                        0.00000595f, 0.00000538f, 0.00000486f, 0.00000440f, 0.00000397f,
                                                        0.00000359f, 0.00000324f, 0.00000292f, 0.00000264f, 0.00000238f,
                                                        0.00000214f, 0.00000193f, 0.00000174f, 0.00000157f, 0.00000141f,
                                                        0.00000127f, 0.00000114f, 0.00000103f, 0.00000092f, 0.00000083f,
                                                        0.00000074f, 0.00000067f, 0.00000060f, 0.00000054f, 0.00000048f,
                                                        0.00000043f, 0.00000039f, 0.00000035f, 0.00000031f, 0.00000028f,
                                                        0.00000025f, 0.00000022f, 0.00000020f, 0.00000018f, 0.00000016f,
                                                        0.00000014f, 0.00000013f, 0.00000011f, 0.00000010f, 0.00000009f,
                                                        0.00000008f, 0.00000007f, 0.00000006f, 0.00000006f, 0.00000005f,
                                                        0.00000004f, 0.00000004f, 0.00000003f, 0.00000003f, 0.00000003f,
                                                        0.00000002f};

// Where the above if lhoodf, this a hard coded/optimized version:
//ratio = 0.99; r_min =0;  	r_max = 3;
//lhood = ratio/(r_max -r_min)  + (1-ratio)*lhood ; hard_coded_log_lhood=log(lhood)
float hard_coded_log_lhood[] = {-1.0614388f, -1.0614480f, -1.0614757f, -1.0615217f, -1.0615862f, -1.0616689f, -1.0617698f, -1.0618887f, -1.0620256f, -1.0621803f, -1.0623526f, -1.0625423f, -1.0627491f, -1.0629730f, -1.0632135f, -1.0634705f, -1.0637437f, -1.0640327f, -1.0643372f, -1.0646569f, -1.0649914f, -1.0653405f, -1.0657036f, -1.0660804f, -1.0664705f, -1.0668735f, -1.0672889f, -1.0677164f, -1.0681554f, -1.0686054f, -1.0690662f, -1.0695370f, -1.0700176f, -1.0705073f, -1.0710057f, -1.0715124f, -1.0720267f, -1.0725482f, -1.0730764f, -1.0736108f, -1.0741509f, -1.0746962f, -1.0752462f, -1.0758003f, -1.0763581f, -1.0769191f, -1.0774827f, -1.0780486f, -1.0786162f, -1.0791851f, -1.0797547f, -1.0803247f, -1.0808945f, -1.0814638f, -1.0820321f, -1.0825989f, -1.0831639f, -1.0837267f, -1.0842868f, -1.0848439f, -1.0853977f, -1.0859476f, -1.0864935f, -1.0870350f, -1.0875718f, -1.0881035f, -1.0886298f, -1.0891506f, -1.0896655f, -1.0901742f, -1.0906766f, -1.0911723f, -1.0916613f, -1.0921433f, -1.0926181f, -1.0930855f, -1.0935454f, -1.0939976f, -1.0944421f, -1.0948787f, -1.0953073f, -1.0957277f, -1.0961400f, -1.0965441f, -1.0969398f, -1.0973272f, -1.0977063f, -1.0980769f, -1.0984391f, -1.0987930f, -1.0991384f, -1.0994755f, -1.0998042f, -1.1001246f, -1.1004367f, -1.1007407f, -1.1010364f, -1.1013241f, -1.1016038f, -1.1018756f, -1.1021396f, -1.1023958f, -1.1026444f, -1.1028855f, -1.1031191f, -1.1033454f, -1.1035646f, -1.1037767f, -1.1039819f, -1.1041802f, -1.1043719f, -1.1045570f, -1.1047358f, -1.1049082f, -1.1050746f, -1.1052349f, -1.1053894f, -1.1055382f, -1.1056815f, -1.1058193f, -1.1059518f, -1.1060792f, -1.1062016f, -1.1063192f, -1.1064320f, -1.1065402f, -1.1066440f, -1.1067435f, -1.1068389f, -1.1069302f, -1.1070176f, -1.1071012f, -1.1071811f, -1.1072575f, -1.1073306f, -1.1074003f, -1.1074668f, -1.1075303f, -1.1075909f, -1.1076486f, -1.1077036f, -1.1077560f, -1.1078059f, -1.1078533f, -1.1078985f, -1.1079414f, -1.1079821f, -1.1080208f, -1.1080576f, -1.1080925f, -1.1081256f, -1.1081569f, -1.1081867f, -1.1082148f, -1.1082415f, -1.1082667f, -1.1082906f, -1.1083132f, -1.1083345f, -1.1083547f, -1.1083737f, -1.1083917f, -1.1084086f, -1.1084246f, -1.1084397f, -1.1084538f, -1.1084672f, -1.1084798f, -1.1084917f, -1.1085028f, -1.1085133f, -1.1085231f, -1.1085324f, -1.1085411f, -1.1085492f, -1.1085569f, -1.1085640f, -1.1085707f, -1.1085770f, -1.1085829f, -1.1085885f, -1.1085936f, -1.1085985f, -1.1086030f, -1.1086072f, -1.1086111f, -1.1086148f, -1.1086183f, -1.1086215f, -1.1086245f, -1.1086272f, -1.1086298f, -1.1086323f, -1.1086345f, -1.1086366f, -1.1086385f, -1.1086404f, -1.1086420f, -1.1086436f, -1.1086451f, -1.1086464f, -1.1086477f, -1.1086488f, -1.1086499f, -1.1086509f, -1.1086518f, -1.1086527f, -1.1086534f, -1.1086542f, -1.1086549f, -1.1086555f, -1.1086561f, -1.1086566f, -1.1086571f, -1.1086575f, -1.1086580f, -1.1086583f, -1.1086587f, -1.1086590f, -1.1086593f, -1.1086596f, -1.1086599f, -1.1086601f, -1.1086603f, -1.1086605f, -1.1086607f, -1.1086609f, -1.1086610f, -1.1086611f, -1.1086613f, -1.1086614f, -1.1086615f, -1.1086616f, -1.1086617f, -1.1086618f, -1.1086619f, -1.1086619f, -1.1086620f, -1.1086620f, -1.1086621f, -1.1086621f, -1.1086622f, -1.1086622f, -1.1086623f, -1.1086623f, -1.1086623f, -1.1086624f, -1.1086624f, -1.1086624f, -1.1086624f, -1.1086624f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086625f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f, -1.1086626f};

// Disparity:
// sigma 0.025
double top_lookup[]={15.9577, 15.8165, 15.4003, 14.7308, 13.8422, 12.7779, 11.5877, 10.3231, 9.0345, 7.7674, 6.5604, 5.4433, 4.4368, 3.5527, 2.7947, 2.1596, 1.6395, 1.2227, 0.89578, 0.64471, 0.45584, 0.31662, 0.21604, 0.14482, 0.095364, 0.061691, 0.039205, 0.024476, 0.015011, 0.0090444, 0.0053532, 0.0031126, 0.001778, 0.0009977, 0.00054999, 0.00029784, 0.00015845, 8.2811e-05, 4.2517e-05, 2.1444e-05, 1.0625e-05, 5.1718e-06, 2.473e-06, 1.1617e-06, 5.361e-07, 2.4304e-07, 1.0824e-07, 4.7354e-08, 2.0353e-08, 8.5933e-09, 3.5644e-09, 1.4524e-09, 5.8138e-10, 2.2862e-10, 8.832e-11, 3.3518e-11, 1.2496e-11, 4.5766e-12, 1.6466e-12, 5.8201e-13, 2.0209e-13, 6.8935e-14, 2.31e-14, 7.6043e-15, 2.4592e-15, 7.8127e-16, 2.4383e-16, 7.4758e-17, 2.2517e-17, 6.6624e-18, 1.9366e-18, 5.5299e-19, 1.5512e-19, 4.2749e-20, 1.1573e-20, 3.0778e-21, 8.0413e-22, 2.0639e-22, 5.2038e-23, 1.289e-23, 3.1365e-24, 7.4975e-25, 1.7606e-25, 4.0617e-26, 9.2049e-27, 2.0493e-27, 4.4821e-28, 9.6302e-29, 2.0327e-29, 4.2148e-30, 8.5855e-31, 1.718e-31, 3.3774e-32, 6.5224e-33, 1.2374e-33, 2.3062e-34, 4.2225e-35, 7.5947e-36, 1.3419e-36, 2.3294e-37, 3.9721e-38, 6.6539e-39, 1.095e-39, 1.7703e-40, 2.8115e-41, 4.3864e-42, 6.7231e-43, 1.0123e-43, 1.4973e-44, 2.1758e-45, 3.1059e-46, 4.3555e-47, 6.0003e-48, 8.1205e-49, 1.0796e-49, 1.4101e-50, 1.8092e-51, 2.2804e-52, 2.8237e-53, 3.4349e-54, 4.1047e-55, 4.8186e-56, 5.5571e-57, 6.2958e-58, 7.007e-59, 7.6611e-60, 8.2287e-61, 8.6827e-62, 9.0002e-63, 9.165e-64, 9.1683e-65, 9.01e-66, 8.6984e-67, 8.2497e-68, 7.6862e-69, 7.035e-70, 6.3255e-71, 5.5874e-72, 4.8484e-73, 4.133e-74, 3.4611e-75, 2.8474e-76, 2.3012e-77, 1.827e-78, 1.425e-79, 1.0918e-80, 8.2183e-82, 6.077e-83, 4.4144e-84, 3.1502e-85, 2.2084e-86, 1.5209e-87, 1.029e-88, 6.8387e-90, 4.4651e-91, 2.864e-92, 1.8046e-93, 1.1171e-94, 6.793e-96, 4.058e-97, 2.3815e-98, 1.373e-99, 7.7759e-101, 4.3264e-102, 2.3647e-103, 1.2697e-104, 6.6975e-106, 3.4706e-107, 1.7667e-108, 8.8352e-110, 4.3405e-111, 2.0948e-112, 9.9319e-114, 4.6259e-115, 2.1166e-116, 9.514e-118, 4.2011e-119, 1.8224e-120, 7.7661e-122, 3.2512e-123, 1.3371e-124, 5.402e-126, 2.144e-127, 8.3597e-129, 3.202e-130, 1.2049e-131, 4.4538e-133, 1.6173e-134, 5.7697e-136, 2.022e-137, 6.9614e-139, 2.3544e-140, 7.8227e-142, 2.5533e-143, 8.1871e-145, 2.5789e-146, 7.9803e-148, 2.426e-149, 7.2448e-151, 2.1255e-152, 6.1257e-154, 1.7343e-155, 4.8239e-157, 1.3181e-158, 3.538e-160, 9.3294e-162, 2.4167e-163, 6.1502e-165, 1.5375e-166, 3.7761e-168, 9.1103e-170, 2.1593e-171, 5.0276e-173, 1.15e-174, 2.5841e-176, 5.7042e-178, 1.237e-179, 2.6352e-181, 5.5149e-183, 1.1338e-184, 2.29e-186, 4.5436e-188, 8.8561e-190, 1.6958e-191, 3.1899e-193, 5.8946e-195, 1.0701e-196, 1.9083e-198, 3.3433e-200, 5.7541e-202, 9.7287e-204, 1.6159e-205, 2.6366e-207, 4.2263e-209, 6.6552e-211, 1.0295e-212, 1.5645e-214, 2.3357e-216, 3.4256e-218, 4.9354e-220, 6.9855e-222, 9.7128e-224, 1.3267e-225, 1.7803e-227, 2.3468e-229, 3.039e-231, 3.8662e-233, 4.8318e-235, 5.9321e-237, 7.1548e-239, 8.4773e-241, 9.8673e-243, 1.1283e-244, 1.2674e-246, 1.3986e-248, 1.5162e-250, 1.6147e-252, 1.6893e-254, 1.7362e-256, 1.753e-258, 1.7388e-260, 1.6942e-262, 1.6218e-264, 1.525e-266, 1.4088e-268, 1.2785e-270, 1.1398e-272, 9.9826e-275, 8.5888e-277, 7.2594e-279, 6.0276e-281, 4.9167e-283, 3.9398e-285, 3.1014e-287, 2.3984e-289, 1.8221e-291, 1.3598e-293, 9.9699e-296, 7.1808e-298, 5.0808e-300, 3.5316e-302, 2.4115e-304, 1.6177e-306, 1.066e-308, 6.9011e-311, 4.3889e-313, 2.742e-315, 1.6829e-317, 1.0147e-319, 6.324e-322, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float bottom_lookup[]={0.5f, 0.55304f, 0.60514f, 0.65542f, 0.7031f, 0.74751f, 0.78814f, 0.82468f, 0.85694f, 0.88493f, 0.90879f, 0.92877f, 0.9452f, 0.95848f, 0.96903f, 0.97725f, 0.98355f, 0.98829f, 0.9918f, 0.99435f, 0.99617f, 0.99744f, 0.99832f, 0.99892f, 0.99931f, 0.99957f, 0.99974f, 0.99984f, 0.99991f, 0.99994f, 0.99997f, 0.99998f, 0.99999f, 0.99999f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.99999f, 0.99999f, 0.99998f, 0.99997f, 0.99994f, 0.99991f, 0.99984f, 0.99974f, 0.99957f, 0.99931f, 0.99892f, 0.99832f, 0.99744f, 0.99617f, 0.99435f, 0.9918f, 0.98829f, 0.98355f, 0.97725f, 0.96903f, 0.95848f, 0.9452f, 0.92877f, 0.90879f, 0.88493f, 0.85694f, 0.82468f, 0.78814f, 0.74751f, 0.7031f, 0.65542f, 0.60514f, 0.55304f, 0.5f};

using namespace pcl::simulation;

// Finds the maximum level n so a and b are still
// divisible by 2^n
int
max_level (int a, int b)
{
  int level = 0;
  while (true)
  {
    if (a%2 || b%2) return level;
    a = a / 2;
    b = b / 2;
    level++;
  }
}

// display_tic_toc: a helper function which accepts a set of
// timestamps and displays the elapsed time between them as
// a fraction and time used [for profiling]
void
display_tic_toc (vector<double> &tic_toc,const string &fun_name)
{
  size_t tic_toc_size = tic_toc.size ();

  double percent_tic_toc_last = 0;
  double dtime = tic_toc[tic_toc_size-1] - tic_toc[0];
  cout << "fraction_" << fun_name << ",";
  for (size_t i = 0; i < tic_toc_size; i++)
  {
    double percent_tic_toc =  (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (size_t i = 0; i < tic_toc_size; i++)
  {
    double percent_tic_toc = (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";
}

pcl::simulation::RangeLikelihood::RangeLikelihood (int rows, int cols, int row_height, int col_width, Scene::Ptr scene) :
      scene_(scene), rows_(rows), cols_(cols), row_height_(row_height), col_width_(col_width),
      depth_buffer_dirty_(true),
      color_buffer_dirty_(true),
      score_buffer_dirty_(true),
      fbo_ (0),
      depth_render_buffer_ (0),
      color_render_buffer_ (0),
      depth_texture_ (0),
      score_texture_ (0),
      score_summarized_texture_ (0),
      sensor_texture_ (0),
      likelihood_texture_ (0),
      compute_likelihood_on_cpu_ (false),
      aggregate_on_cpu_ (false),
      use_instancing_ (false),
      use_color_ (true),
      sum_reduce_ (cols * col_width, rows * row_height, max_level (col_width, row_height))
{
  height_ = rows_ * row_height;
  width_ = cols_ * col_width;

  depth_buffer_ = new float[width_*height_];
  color_buffer_ = new uint8_t[width_*height_*3];

  // Set Default Camera Intrinstic Parameters. techquad
  // Correspond closely to those stated here:
  // http://www.ros.org/wiki/kinect_calibration/technical
  camera_width_ = 640;
  camera_height_ = 480;
  camera_fx_ = 576.09757860f;
  camera_fy_ = 576.09757860f;
  camera_cx_ = 321.06398107f;
  camera_cy_ = 242.97676897f;

  z_near_ = 0.7f;
  z_far_ = 20.0f;
  
  which_cost_function_ = 2; // default to commonly used meter based function
  
  // default lhood parameters - these should always be set by the user
  // so might want to add to constructor eventually:
  sigma_ = 0.1;
  floor_proportion_ = 0.9; 

  int height = rows * row_height;
  int width = cols * col_width;

  // For now we only support a limited size texture
  assert (height >0 && height <= 8192 && width > 0 && width <= 8192);
  // throw std::runtime_error "

  // Allocate framebuffer
  glGenRenderbuffers (1, &depth_render_buffer_);
  glBindRenderbuffer (GL_RENDERBUFFER, depth_render_buffer_);
  glRenderbufferStorage (GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width, height);
  glBindRenderbuffer (GL_RENDERBUFFER, 0);

  glGenRenderbuffers (1, &color_render_buffer_);
  glBindRenderbuffer (GL_RENDERBUFFER, color_render_buffer_);
  glRenderbufferStorage (GL_RENDERBUFFER, GL_RGB8UI, width, height);
  glBindRenderbuffer (GL_RENDERBUFFER, 0);

  // Setup texture to store depth image
  glGenTextures (1, &depth_texture_);
  glBindTexture (GL_TEXTURE_2D, depth_texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
  glBindTexture (GL_TEXTURE_2D, 0);

  glGenTextures (1, &color_texture_);
  glBindTexture (GL_TEXTURE_2D, color_texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  glBindTexture (GL_TEXTURE_2D, 0);

  // Setup texture for incoming image
  glGenTextures (1, &sensor_texture_);
  glBindTexture (GL_TEXTURE_2D, sensor_texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, col_width, row_height, 0, GL_RED, GL_FLOAT, NULL);
  glBindTexture (GL_TEXTURE_2D, 0);

  // Texture for to score on each pixel
  glGenTextures (1, &score_texture_);
  glBindTexture (GL_TEXTURE_2D, score_texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, width_, height_, 0, GL_RED, GL_FLOAT, NULL);
  glBindTexture (GL_TEXTURE_2D, 0);

  // Setup texture for likelihood function
  // size of likelihood texture
  int likelihood_size = 301;
  glActiveTexture (GL_TEXTURE2);
  glGenTextures (1, &likelihood_texture_);
  glBindTexture (GL_TEXTURE_2D, likelihood_texture_);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_RED);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, likelihood_size, 1, 0, GL_RED, GL_FLOAT, normal_sigma0x5_normal1x0_range0to3_step0x01);
  glBindTexture (GL_TEXTURE_2D, 0);

  // Setup the framebuffer object for rendering
  glGenFramebuffers (1, &fbo_);
  glBindFramebuffer (GL_FRAMEBUFFER, fbo_);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture_, 0);
  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  if (gllib::getGLError() != GL_NO_ERROR)
  {
    std::cerr << "RangeLikelihoodGLSL::RangeLikelihoodGLSL: Failed initializing OpenGL buffers" << std::endl;
    exit(-1);
  }

  glGenFramebuffers (1, &score_fbo_);
  glBindFramebuffer (GL_FRAMEBUFFER, score_fbo_);
  glFramebufferRenderbuffer (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, score_texture_, 0);
  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  // Load shader
  likelihood_program_ = gllib::Program::Ptr (new gllib::Program ());
  // TODO: to remove file dependency include the shader source in the binary
  if (!likelihood_program_->addShaderFile ("compute_score.vert", gllib::VERTEX))
  {
    std::cout << "Failed loading vertex shader" << std::endl;
    exit (-1);
  }

  if (!likelihood_program_->addShaderFile ("compute_score.frag", gllib::FRAGMENT))
  {
    std::cout << "Failed loading fragment shader" << std::endl;
    exit (-1);
  }

  likelihood_program_->link ();

  vertices_.push_back (Eigen::Vector3f (-1.0,  1.0, 0.0));
  vertices_.push_back (Eigen::Vector3f ( 1.0,  1.0, 0.0));
  vertices_.push_back (Eigen::Vector3f ( 1.0, -1.0, 0.0));
  vertices_.push_back (Eigen::Vector3f (-1.0, -1.0, 0.0));

  glGenBuffers (1, &quad_vbo_);
  glBindBuffer (GL_ARRAY_BUFFER, quad_vbo_);
  glBufferData (GL_ARRAY_BUFFER, sizeof (Eigen::Vector3f) * vertices_.size (), &(vertices_[0]), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  gllib::getGLError ();

  // Go back to the default pipeline
  glUseProgram (0);

  score_buffer_ = new float[width_*height_];

}

pcl::simulation::RangeLikelihood::~RangeLikelihood ()
{
  glDeleteBuffers (1, &quad_vbo_);
  glDeleteTextures (1, &depth_texture_);
  glDeleteTextures (1, &color_texture_);
  glDeleteTextures (1, &score_texture_);
  glDeleteTextures (1, &score_summarized_texture_);
  glDeleteTextures (1, &sensor_texture_);
  glDeleteTextures (1, &likelihood_texture_);
  glDeleteFramebuffers (1, &fbo_);
  glDeleteFramebuffers (1, &score_fbo_);
  glDeleteRenderbuffers (1, &depth_render_buffer_);
  glDeleteRenderbuffers (1, &color_render_buffer_);

  delete [] depth_buffer_;
  delete [] color_buffer_;
  delete [] score_buffer_;
}

double
pcl::simulation::RangeLikelihood::sampleNormal (double sigma)
{
  typedef boost::normal_distribution<double> Normal;
  Normal dist (0.0, sigma);
  boost::variate_generator<boost::minstd_rand&, Normal> norm (generator, dist);
  return (norm ());
}

void
pcl::simulation::RangeLikelihood::setupProjectionMatrix ()
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  // Prepare scaled simulated camera projection matrix
  float sx = static_cast<float> (camera_width_) / static_cast<float> (col_width_);
  float sy = static_cast<float> (camera_height_) / static_cast<float> (row_height_);
  float width = static_cast<float> (col_width_);
  float height = static_cast<float> (row_height_);

  float fx = camera_fx_/sx;
  float fy = camera_fy_/sy;
  float cx = camera_cx_/sx;
  float cy = camera_cy_/sy;
  float m[16];
  float z_nf = (z_near_-z_far_);

  m[0] = 2.0f*fx/width;  m[4] = 0;              m[ 8] = 1.0f-(2*cx/width);      m[12] = 0;
  m[1] = 0;              m[5] = 2.0f*fy/height; m[ 9] = 1.0f-(2*cy/height);     m[13] = 0;
  m[2] = 0;              m[6] = 0;              m[10] = (z_far_+z_near_)/z_nf;  m[14] = 2.0f*z_near_*z_far_/z_nf;
  m[3] = 0;              m[7] = 0;              m[11] = -1.0f;                  m[15] = 0;
  glMultMatrixf (m);
}

void
pcl::simulation::RangeLikelihood::applyCameraTransform (const Eigen::Isometry3d & pose)
{
  float T[16];
  Eigen::Matrix4f m = (pose.matrix ().inverse ()).cast<float> ();
  T[0] = m(0,0); T[4] = m(0,1); T[8] = m(0,2); T[12] = m(0,3);
  T[1] = m(1,0); T[5] = m(1,1); T[9] = m(1,2); T[13] = m(1,3);
  T[2] = m(2,0); T[6] = m(2,1); T[10] = m(2,2); T[14] = m(2,3);
  T[3] = m(3,0); T[7] = m(3,1); T[11] = m(3,2); T[15] = m(3,3);
  glMultMatrixf(T);
}

void
pcl::simulation::RangeLikelihood::drawParticles (std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses)
{
  int n = 0;
  for (int i=0; i<rows_; ++i)
  {
    for (int j=0; j<cols_; ++j)
    {
      glMatrixMode (GL_MODELVIEW);
      glLoadIdentity ();

      glViewport(j*col_width_, i*row_height_, col_width_, row_height_);

      // Go from Z-up, X-forward coordinate frame
      // to OpenGL Z-out,Y-up [both Right Handed]
      float T[16];
      T[0] =  0;   T[4] = -1.0; T[8] =  0; T[12] = 0;
      T[1] =  0;   T[5] = 0;    T[9] =  1; T[13] = 0;
      T[2] = -1.0; T[6] = 0;    T[10] = 0; T[14] = 0;
      T[3] =  0;   T[7] = 0;    T[11] = 0; T[15] = 1;
      glMultMatrixf (T);

      // Apply camera transformation
      applyCameraTransform (poses[n++]);

      // Draw the planes in each location:
      scene_->draw ();
    }
  }
}


/////////////////////////////////////////////////////////////////
// Below are 4 previously used cost functions:
// 0 original scoring method
float
costFunction0 (float ref_val,float depth_val)
{
  return  (sqr(ref_val - depth_val));
}

// 1st working cost function:
// Empirical reverse mapping between depthbuffer and true depth:
// Version 0: [25 aug 2011]
// TRUEDEPTH =  1/(1.33 -(DEPTHBUFFER)*1.29)
//float cost = sqr(ref[col%col_width] - 1/(1.33 -(*depth)*1.29));
// Version 1: [29 aug 2011] Exact version using correct mappings:
float
costFunction1 (float ref_val, float depth_val)
{
  float cost = sqr (ref_val - 1/(1.4285f - (depth_val)*1.3788f));
  //std::cout << " [" << ref_val << "," << 1/(1.4285 -(depth_val)*1.3788) << "] ";
  if (ref_val < 0)
  { // all images pixels with no range
    cost =1;
  }
  if (cost > 10)
  { // required to lessen the effect of modelpixel with no range (ie holes in the model)
    cost = 10;
  }
  //return log (cost);
  return (cost);
}

// 1st working likelihood function (by far most commonly used)
float
costFunction2 (float ref_val, float depth_val)
{
  float min_dist = abs(ref_val - 1/(1.4285f - (depth_val)*1.3788f));
  int lup = static_cast<int> (ceil (min_dist*100)); // has resolution of 0.01m

  if (lup > 300)
  { // implicitly this caps the cost if there is a hole in the model
    lup = 300;
  }
  
  double lhood = 1;
  if (pcl_isnan (depth_val))
  { // pixels with nan depth - for openNI null points
    lhood = 1; // log(1) = 0 ---> has no effect
  }
  else if(ref_val < 0)
  { // all RGB pixels with no depth - for freenect null points
    lhood = 1; // log(1) = 0 ---> has no effect
  }
  else
  {
    lhood = normal_sigma0x5_normal1x0_range0to3_step0x01[lup];
    // add a ground floor:
    // increasing this will mean that the likelihood is less peaked
    // but you need more particles to do this...
    // with ~90particles user 0.999, for example in the quad dataset
    // ratio of uniform to	normal
    double ratio = 0.99;//was always 0.99; 
    double r_min = 0; // metres
    double r_max = 3; // metres
    lhood = ratio/(r_max -r_min) + (1-ratio)*lhood ;
  }
  return static_cast<float> (log (lhood));
}

float
costFunction3 (float ref_val,float depth_val)
{
  float log_lhood=0;
  // log(1) = 0 ---> has no effect
  if (ref_val < 0)
  {
    // all images pixels with no range
  }
  else if (ref_val > 7)
  {
    // ignore long ranges... for now
  }
  else
  { // working range
    float min_dist = abs (ref_val - 0.7253f/(1.0360f - (depth_val)));

    int lup = static_cast<int> (ceil (min_dist*100)); // has resulution of 0.01m
    if (lup > 300)
    { // implicitly this caps the cost if there is a hole in the model
      lup = 300;
    }
    log_lhood = hard_coded_log_lhood[lup];
  }
  return log_lhood;
}

float
costFunction4(float ref_val,float depth_val)
{
  float disparity_diff = abs( ( -0.7253f/ref_val +1.0360f ) -  depth_val );

  int top_lup = static_cast<int> (ceil (disparity_diff*300)); // has resulution of 0.001m
  if (top_lup > 300)
  {
    top_lup =300;
  }
  float top = static_cast<float> (top_lookup[top_lup]);// round( abs(x-mu) *1000+1) );

  // bottom:
  //bottom = bottom_lookup(   round(mu*1000+1));
  int bottom_lup = static_cast<int> (ceil( (depth_val) * 300)); // has resulution of 0.001m
  if (bottom_lup > 300)
  {
    bottom_lup =300;
  }
  float bottom = bottom_lookup[bottom_lup];// round( abs(x-mu) *1000+1) );

  float proportion = 0.999f;
  float lhood = proportion + (1-proportion)*(top/bottom);

  // safety fix thats seems to be required due to opengl ayschronizate
  // ask hordur about this
  if (bottom == 0)
  {
    lhood = proportion;
  }

  if (ref_val< 0)
  { // all images pixels with no range
    lhood = 1; // log(1) = 0 ---> has no effect
  }
  return log(lhood);
}

// TODO: WHEN WE'RE HAPPY THIS SHOULD BE "THE" LIKELIHOOD FUNCTION
// add these global variables into the class
// abd use sigma and floor_proportion directly from class also
using boost::math::normal; // typedef provides default type is double.
normal unit_norm_dist(0,1); // (default mean = zero, and standard deviation = unity)
double costFunction5(double measured_depth, double model_disp, double sigma, double floor_proportion)
{
  // NEED TO CONVERT MEASURED TO DISPARITY
  double measured_disp =  (-0.7253/measured_depth + 1.0360 );

  // measured_depth = ref_val  [m]
  // model_disp = depth_val [0-1]
  // upper and lower bound on depth buffer:
  double lower_bound =0;
  double upper_bound =1;

  double gaussian_part = pdf(unit_norm_dist, (measured_disp-model_disp)/sigma)/sigma;
  double truncation = 1/cdf(unit_norm_dist,(upper_bound-model_disp)/sigma) -   cdf(unit_norm_dist, (lower_bound-model_disp)/sigma);
	
  double trunc_gaussian_part = truncation*gaussian_part;

  double lhood= (floor_proportion/(upper_bound-lower_bound) + (1-floor_proportion)*trunc_gaussian_part);

  if (measured_depth< 0){ // all images pixels with no range
    lhood = 1; // log(1) = 0 ---> has no effect
  }

  return log (lhood);
}

void
pcl::simulation::RangeLikelihood::computeScores (float* reference,
                                                 std::vector<float> & scores)
{
  const float* depth = getDepthBuffer();
  // Mapping between disparity and range:
  // range or depth = 1/disparity
  //
  // the freenect produces a disparity <here we call this depth_buffer>
  // that is mapped between 0->1 to minimize quantization
  // near_range = n = 0.7m   | far_range = f = 20m
  // disparity can be found as a linear function of the depth_buffer (d = [0,1] )
  // disparity =  1/n   - (f-n)*d / (n*f)
  // Below We compare range-versus-range using this mapping
  //
  // TODO: remove usage of 'depth' and 'depth_buffer_' as variable names as it implies
  // that that is was held by these variables

  // ref[col%col_width] - z/depth value in metres,   0-> ~20
  // depth_val - contents of depth buffer [0->1]
  
  // for row across each image in a row of model images
  for (int row = 0; row < rows_*row_height_; row++)
  {
    float* ref = reference + col_width_*(row % row_height_);
    // for each column: across each image in a column of model images
    for (int col = 0; col < cols_*col_width_; col++)
    {
      float depth_val = (*depth++); // added jan 2012 - check this is not a breaking fix later mfallon
      float score = 0;
      if (which_cost_function_ == 0)
      {
        score = costFunction0 (ref[col%col_width_],depth_val);
      }
      else if (which_cost_function_ == 1)
      {
	score = costFunction1 (ref[col%col_width_],depth_val);
      }
      else if (which_cost_function_ == 2)
      {
        score = costFunction2 (ref[col%col_width_],depth_val);
      }
      else if(which_cost_function_==3)
      {
	score = costFunction3 (ref[col%col_width_],depth_val);
      }
      else if (which_cost_function_ == 4)
      {
        score = costFunction4 (ref[col%col_width_],depth_val);
      }
      else if (which_cost_function_ == 5)
      {
	//double sigma = 0.025;
	//double floor_proportion_ = 0.999;
        score = static_cast<float> (costFunction5 (ref[col%col_width_],depth_val,sigma_,floor_proportion_));
      }
      scores[row/row_height_ * cols_ + col/col_width_] += score;
      //std::cout << "(" << scores[row/row_height_ * cols_ + col/col_width_] <<"," << score << "," << ref[col%col_width_] << "," << depth_val << ") ";
      score_buffer_[row*width_ + col] =  score;
    }
  }
  score_buffer_dirty_ = false;
}

void
pcl::simulation::RangeLikelihood::getPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
  bool make_global,
  const Eigen::Isometry3d & pose)
{
  // TODO: check if this works for for rows/cols >1  and for width&height != 640x480
  // i.e. multiple tiled images
  pc->width    = col_width_;
  pc->height   = row_height_;
  // Was:
  //pc->width    = camera_width_;
  //pc->height   = camera_height_;

  pc->is_dense = false;
  pc->points.resize (pc->width*pc->height);

  int points_added = 0;

  float camera_fx_reciprocal_ = 1.0f / camera_fx_;
  float camera_fy_reciprocal_ = 1.0f / camera_fy_;
  float zn = z_near_;
  float zf = z_far_;

  const uint8_t* color_buffer = getColorBuffer();

  // TODO: support decimation
  // Copied the format of RangeImagePlanar::setDepthImage()
  // Use this as a template for decimation
  for (int y = 0; y < row_height_ ; ++y) //camera_height_
  {
    for (int x = 0; x < col_width_ ; ++x)  // camera_width_
    {
      // Find XYZ from normalized 0->1 mapped disparity
      int idx = points_added; // y*camera_width_ + x;
      float d = depth_buffer_[y*camera_width_ + x] ;
      if (d < 1.0) // only add points with depth buffer less than max (20m) range
      {
        float z = zf*zn/((zf-zn)*(d - zf/(zf-zn)));

        // TODO: add mode to ignore points with no return i.e. depth_buffer_ ==1
        // NB: OpenGL uses a Right Hand system with +X right, +Y up, +Z back out of the screen,
        // The Z-buffer is natively -1 (far) to 1 (near)
        // But in this class we invert this to be 0 (near, 0.7m) and 1 (far, 20m)
        // ... so by negating y we get to a right-hand computer vision system
        // which is also used by PCL and OpenNi
        pc->points[idx].z = z;
        pc->points[idx].x = (static_cast<float> (x)-camera_cx_) * z * (-camera_fx_reciprocal_);
        pc->points[idx].y = (static_cast<float> (y)-camera_cy_) * z * (-camera_fy_reciprocal_);

	int rgb_idx = y*col_width_ + x;  //camera_width_
        pc->points[idx].b = color_buffer[rgb_idx*3+2]; // blue
        pc->points[idx].g = color_buffer[rgb_idx*3+1]; // green
        pc->points[idx].r = color_buffer[rgb_idx*3]; // red
        points_added++;
      }
    }
  }
  pc->width    = 1;
  pc->height   = points_added;
  pc->points.resize (points_added);

  if (make_global)
  {
    // Go from OpenGL to (Z-up, X-forward, Y-left)
    Eigen::Matrix4f T;
    T <<  0, 0, -1, 0,
         -1, 0,  0, 0,
          0, 1,  0, 0,
          0, 0,  0, 1;
    Eigen::Matrix4f m = pose.matrix ().cast<float> ();
    m = m * T;
    pcl::transformPointCloud (*pc, *pc, m);
  }
  else
  {
    // Go from OpenGL to Camera (Z-forward, X-right, Y-down)
    Eigen::Matrix4f T;
    T <<  1,  0,  0, 0,
          0, -1,  0, 0,
          0,  0, -1, 0,
          0,  0,  0, 1;
    pcl::transformPointCloud (*pc, *pc, T);

    // Go from Camera to body (Z-up, X-forward, Y-left)
    Eigen::Matrix4f cam_to_body;
    cam_to_body <<  0,  0, 1, 0,
                   -1,  0, 0, 0,
                    0, -1, 0, 0,
                    0,  0, 0, 1;
    Eigen::Matrix4f camera = pose.matrix ().cast<float> ();
    camera = camera * cam_to_body;
    pc->sensor_origin_ = camera.rightCols (1);
    Eigen::Quaternion<float> quat (camera.block<3,3> (0,0));
    pc->sensor_orientation_ = quat;
  }
}

void
pcl::simulation::RangeLikelihood::getRangeImagePlanar(pcl::RangeImagePlanar &rip)
{
  rip.setDepthImage (depth_buffer_,
    camera_width_,camera_height_, camera_fx_,camera_fy_,
   camera_fx_, camera_fy_);
}
  
void
pcl::simulation::RangeLikelihood::addNoise ()
{
  // Other noises:
  // edge noise: look for edges in the range image and add a few pixels here and there
  // texture noise: look at the normals and

  // Add Gaussian Noise
  // TODO: make the variance a parameter
  // TODO: might want to add a range-based variance
  float variance = 0.0015f;
  for (int i = 0; i < camera_width_*camera_height_ ; i++)
  {
    if (depth_buffer_[i] < 1)
    {
      depth_buffer_[i] = depth_buffer_[i] + variance * static_cast<float> (sampleNormal ());
      if (depth_buffer_[i] > 1)
      {
        depth_buffer_[i] = 1.0;
      }
      else if (depth_buffer_[i] < 0)
      {
        depth_buffer_[i] = 0.0;
      }
    }
  }

  // Add Kinect/Primesense Quantisation Noise:
  // TODO: better fit this:
  // 0.6m  = ~600 kinect return
  // 20m   = ~1070 kinect return - not not well calibrated
  // The fitted model stated here cannot work for long ranges:
  // http://www.ros.org/wiki/kinect_calibration/technical
  // TODO: make a parameter
  float bins = 470;
  for (int i = 0; i < camera_width_*camera_height_ ; i++)
  {
    depth_buffer_[i] =  ceil (depth_buffer_[i]*bins)/bins;
  }
  cout << "in add noise\n";
}

void
RangeLikelihood::computeLikelihoods (float* reference,
                                     std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses,
                                     std::vector<float> & scores)
{
  #if DO_TIMING_PROFILE
    vector<double> tic_toc;
    tic_toc.push_back(getTime());
  #endif  
  
  scores.resize (cols_*rows_);
  std::fill (scores.begin (), scores.end (), 0);
  
  // Generate depth image for each particle
  render (poses);
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back (getTime ());
  #endif

  #if DO_TIMING_PROFILE
    tic_toc.push_back (getTime ());
  #endif  
  // The depth image is now in depth_texture_

  // Compute likelihoods
  if (compute_likelihood_on_cpu_)
  {
    computeScores (reference, scores);
  }
  else
  {
    computeScoresShader (reference);
    
    // Aggregate results (we do not use GPU to sum cpu scores)
    if (aggregate_on_cpu_)
    {
      const float* score_buffer = getScoreBuffer();

      for (int n = 0, row = 0; row < height_; ++row)
      {
        for (int col = 0; col < width_; ++col, ++n)
        {
          scores[row/row_height_ * cols_ + col/col_width_] += score_buffer[n];
        }
      }
    }
    else
    {
      int levels = max_level (row_height_, col_width_);
      int reduced_width = width_ >> levels;
      int reduced_height = height_ >> levels;
      int reduced_col_width = col_width_ >> levels;
      int reduced_row_height = row_height_ >> levels;

      float* score_sum = new float[reduced_width * reduced_height];
      sum_reduce_.sum (score_texture_, score_sum);
      for (int n = 0, row = 0; row < reduced_height; ++row)
      {
        for (int col = 0; col < reduced_width; ++col, ++n)
        {
          scores[row/reduced_row_height * cols_ + col/reduced_col_width] += score_sum[n];
        }
      }
      delete [] score_sum;
    }
  }
  
  #if DO_TIMING_PROFILE
    tic_toc.push_back (getTime ());
    display_tic_toc (tic_toc, "range_likelihood");
  #endif    
}

// Computes the likelihood scores using a shader
void
pcl::simulation::RangeLikelihood::computeScoresShader (float* reference)
{
  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cout << "GL error: RangeLikelihood::compute_scores_shader - enter" << std::endl;
  }

#ifdef SIMULATION_DEBUG
  std::cout << "DepthSampler location: " << likelihood_program_->getUniformLocation ("DepthSampler") << std::endl;
  std::cout << "ReferenceSampler location: " << likelihood_program_->getUniformLocation ("ReferenceSampler") << std::endl;
  std::cout << "CostSampler location: " << likelihood_program_->getUniformLocation ("CostSampler") << std::endl;

  int depth_tex_id;
  int ref_tex_id;
  int cost_tex_id;

  glGetUniformiv(likelihood_program_->programId (), likelihood_program_->getUniformLocation ("DepthSampler"), &depth_tex_id);
  glGetUniformiv(likelihood_program_->programId (), likelihood_program_->getUniformLocation ("ReferenceSampler"), &ref_tex_id);
  glGetUniformiv(likelihood_program_->programId (), likelihood_program_->getUniformLocation ("CostSampler"), &cost_tex_id);
  std::cout << "depth id: " << depth_tex_id << " " << GL_TEXTURE0 << std::endl;
  std::cout << "ref id: " << ref_tex_id  << " " << GL_TEXTURE1 << std::endl;
  std::cout << "cost id: " << cost_tex_id << " " << GL_TEXTURE2 << std::endl;
#endif

  likelihood_program_->use ();
  likelihood_program_->setUniform ("DepthSampler", 0);
  likelihood_program_->setUniform ("ReferenceSampler", 1);
  likelihood_program_->setUniform ("CostSampler", 2);
  likelihood_program_->setUniform ("cols", cols_);
  likelihood_program_->setUniform ("rows", rows_);
  likelihood_program_->setUniform ("near", z_near_);
  likelihood_program_->setUniform ("far", z_far_);

  glBindFramebuffer (GL_FRAMEBUFFER, score_fbo_);
  glDrawBuffer (GL_COLOR_ATTACHMENT0);

  glReadBuffer (GL_NONE);
  GLboolean enable_depth_test;
  glGetBooleanv (GL_DEPTH_TEST, &enable_depth_test);
  glDisable (GL_DEPTH_TEST);
  glViewport (0, 0, width_, height_);

  // Setup textures
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, depth_texture_);

  glActiveTexture (GL_TEXTURE1);
  glBindTexture (GL_TEXTURE_2D, sensor_texture_);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, col_width_, row_height_, 0, GL_RED, GL_FLOAT, reference);

  glActiveTexture (GL_TEXTURE2);
  glBindTexture (GL_TEXTURE_2D, likelihood_texture_);

  quad_.render ();
  glUseProgram (0);

  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  // Unbind all textures that were used
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, 0);
  glActiveTexture (GL_TEXTURE1);
  glBindTexture (GL_TEXTURE_2D, 0);
  glActiveTexture (GL_TEXTURE2);
  glBindTexture (GL_TEXTURE_2D, 0);

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cout << "GL error: RangeLikelihood::compute_scores_shader - exit" << std::endl;
  }

  if (enable_depth_test == GL_TRUE) glEnable (GL_DEPTH_TEST);

}

void
RangeLikelihood::render (const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > & poses)
{
  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: RangeLikelihood::render - enter" << std::endl;
  }

  GLint old_matrix_mode;
  GLint old_draw_buffer;
  GLint old_read_buffer;
  glGetIntegerv (GL_DRAW_BUFFER, &old_draw_buffer);
  glGetIntegerv (GL_READ_BUFFER, &old_read_buffer);
  glGetIntegerv (GL_MATRIX_MODE, &old_matrix_mode);

  glMatrixMode (GL_PROJECTION);
  glPushMatrix ();

  glMatrixMode (GL_MODELVIEW);
  glPushMatrix ();

  glBindFramebuffer (GL_FRAMEBUFFER, fbo_);

  if (use_color_)
  {
    glDrawBuffer (GL_COLOR_ATTACHMENT0);
  }
  else
  {
    glDrawBuffer (GL_NONE);
  }
  glReadBuffer (GL_NONE);

  GLenum status;
  status = glCheckFramebufferStatus (GL_FRAMEBUFFER);
  switch (status)
  {
    case GL_FRAMEBUFFER_COMPLETE:
    {
      break;
    }
    default:
    {
      std::cout << "RangeLikelihood::render: Framebuffer failed" << std::endl;
      exit (-1);
    }
  }

  // Render
  glPushAttrib (GL_ALL_ATTRIB_BITS);
  glEnable (GL_COLOR_MATERIAL);
  glClearColor (0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth (1.0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Setup projection matrix
  setupProjectionMatrix ();

  glEnable (GL_DEPTH_TEST);
  glDepthMask (GL_TRUE);
  glCullFace (GL_FRONT);
  drawParticles (poses);

  glPopAttrib ();
  glFlush ();

  glBindFramebuffer (GL_FRAMEBUFFER, 0);
  
  // Restore OpenGL state
  glReadBuffer (old_read_buffer);
  glDrawBuffer (old_draw_buffer);

  glMatrixMode (GL_MODELVIEW);
  glPopMatrix ();

  glMatrixMode (GL_PROJECTION);
  glPopMatrix ();

  glMatrixMode (old_matrix_mode);

  if (gllib::getGLError () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: RangeLikelihood::render - exit" << std::endl;
  }

  color_buffer_dirty_ = true;
  depth_buffer_dirty_ = true;
  score_buffer_dirty_ = true;
}

const float*
RangeLikelihood::getDepthBuffer ()
{
  if (depth_buffer_dirty_)
  {
    // Read depth
    glBindFramebuffer (GL_FRAMEBUFFER, fbo_);
    glReadPixels (0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth_buffer_);
    glBindFramebuffer (GL_FRAMEBUFFER, 0);

    if (gllib::getGLError () != GL_NO_ERROR)
    {
      std::cerr << "GL Error: RangeLikelihoodGLSL::getDepthBuffer" << std::endl;
    }

    depth_buffer_dirty_ = false;
  }
  return depth_buffer_;
}

const uint8_t*
RangeLikelihood::getColorBuffer ()
{
  // It's only possible to read the color buffer if it
  // was rendered in the first place.
  assert (use_color_);

  if (color_buffer_dirty_)
  {
    std::cout << "Read color buffer" << std::endl;

    // Read Color
    GLint old_read_buffer;
    glGetIntegerv (GL_READ_BUFFER, &old_read_buffer);

    glBindFramebuffer (GL_FRAMEBUFFER, fbo_);
    glReadBuffer (GL_COLOR_ATTACHMENT0);
    glReadPixels (0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, color_buffer_);
    glBindFramebuffer (GL_FRAMEBUFFER, 0);
    glReadBuffer (old_read_buffer);

    if (gllib::getGLError () != GL_NO_ERROR)
    {
      std::cerr << "GL Error: RangeLikelihood::getColorBuffer" << std::endl;
    }

    color_buffer_dirty_ = false;
  }
  return color_buffer_;
}

// The scores are in score_texture_
const float*
RangeLikelihood::getScoreBuffer ()
{
  if (score_buffer_dirty_ && !compute_likelihood_on_cpu_)
  {
    glActiveTexture (GL_TEXTURE0);
    glBindTexture (GL_TEXTURE_2D, score_texture_);
    glGetTexImage (GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, score_buffer_);
    glBindTexture (GL_TEXTURE_2D, 0);
    if (gllib::getGLError () != GL_NO_ERROR)
    {
      std::cerr << "GL Error: RangeLikelihood::getScoreBuffer" << std::endl;
    }
    score_buffer_dirty_ = false;
  }
  return score_buffer_;
}
