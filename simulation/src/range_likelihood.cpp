#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <pcl/common/time.h>
#include "pcl/simulation/range_likelihood.h"

// For adding noise:
static boost::minstd_rand generator (27u); // seed

//#define SIMULATION_DEBUG 1
#define DO_TIMING_PROFILE 0


using namespace std;

// 301 values, 0.0 uniform  1.0 normal. properly truncated/normalized
float normal_sigma0x5_normal1x0_range0to3_step0x01[] = {1.59576912, 1.59545000, 1.59449302, 1.59289932, 1.59067083,
                                                        1.58781019, 1.58432085, 1.58020697, 1.57547345, 1.57012594,
                                                        1.56417078, 1.55761504, 1.55046646, 1.54273348, 1.53442517,
                                                        1.52555126, 1.51612211, 1.50614865, 1.49564242, 1.48461552,
                                                        1.47308056, 1.46105069, 1.44853953, 1.43556117, 1.42213012,
                                                        1.40826131, 1.39397005, 1.37927201, 1.36418316, 1.34871978,
                                                        1.33289841, 1.31673585, 1.30024906, 1.28345522, 1.26637163,
                                                        1.24901574, 1.23140504, 1.21355714, 1.19548963, 1.17722012,
                                                        1.15876621, 1.14014544, 1.12137525, 1.10247299, 1.08345589,
                                                        1.06434100, 1.04514521, 1.02588518, 1.00657737, 0.98723796,
                                                        0.96788290, 0.94852781, 0.92918802, 0.90987853, 0.89061400,
                                                        0.87140871, 0.85227659, 0.83323116, 0.81428555, 0.79545248,
                                                        0.77674422, 0.75817263, 0.73974913, 0.72148465, 0.70338972,
                                                        0.68547437, 0.66774817, 0.65022022, 0.63289916, 0.61579315,
                                                        0.59890986, 0.58225652, 0.56583986, 0.54966616, 0.53374122,
                                                        0.51807038, 0.50265855, 0.48751015, 0.47262918, 0.45801920,
                                                        0.44368334, 0.42962430, 0.41584438, 0.40234547, 0.38912908,
                                                        0.37619631, 0.36354792, 0.35118428, 0.33910545, 0.32731110,
                                                        0.31580063, 0.30457310, 0.29362725, 0.28296157, 0.27257426,
                                                        0.26246326, 0.25262625, 0.24306068, 0.23376378, 0.22473257,
                                                        0.21596387, 0.20745431, 0.19920035, 0.19119830, 0.18344431,
                                                        0.17593438, 0.16866443, 0.16163022, 0.15482742, 0.14825164,
                                                        0.14189837, 0.13576305, 0.12984106, 0.12412773, 0.11861834,
                                                        0.11330815, 0.10819240, 0.10326630, 0.09852508, 0.09396394,
                                                        0.08957812, 0.08536286, 0.08131342, 0.07742511, 0.07369324,
                                                        0.07011320, 0.06668040, 0.06339032, 0.06023847, 0.05722044,
                                                        0.05433188, 0.05156850, 0.04892611, 0.04640054, 0.04398775,
                                                        0.04168374, 0.03948462, 0.03738655, 0.03538582, 0.03347876,
                                                        0.03166181, 0.02993149, 0.02828442, 0.02671730, 0.02522691,
                                                        0.02381013, 0.02246393, 0.02118538, 0.01997160, 0.01881983,
                                                        0.01772739, 0.01669169, 0.01571021, 0.01478053, 0.01390031,
                                                        0.01306728, 0.01227925, 0.01153414, 0.01082990, 0.01016460,
                                                        0.00953635, 0.00894336, 0.00838388, 0.00785626, 0.00735890,
                                                        0.00689028, 0.00644891, 0.00603340, 0.00564241, 0.00527464,
                                                        0.00492888, 0.00460393, 0.00429869, 0.00401209, 0.00374309,
                                                        0.00349073, 0.00325408, 0.00303227, 0.00282444, 0.00262981,
                                                        0.00244761, 0.00227712, 0.00211766, 0.00196858, 0.00182926,
                                                        0.00169912, 0.00157761, 0.00146420, 0.00135840, 0.00125975,
                                                        0.00116779, 0.00108211, 0.00100231, 0.00092803, 0.00085891,
                                                        0.00079462, 0.00073485, 0.00067930, 0.00062770, 0.00057979,
                                                        0.00053532, 0.00049406, 0.00045581, 0.00042034, 0.00038748,
                                                        0.00035705, 0.00032887, 0.00030280, 0.00027868, 0.00025638,
                                                        0.00023577, 0.00021673, 0.00019915, 0.00018292, 0.00016795,
                                                        0.00015414, 0.00014141, 0.00012968, 0.00011887, 0.00010893,
                                                        0.00009977, 0.00009135, 0.00008360, 0.00007648, 0.00006994,
                                                        0.00006393, 0.00005842, 0.00005336, 0.00004872, 0.00004446,
                                                        0.00004056, 0.00003699, 0.00003372, 0.00003072, 0.00002798,
                                                        0.00002548, 0.00002319, 0.00002110, 0.00001918, 0.00001744,
                                                        0.00001585, 0.00001439, 0.00001307, 0.00001186, 0.00001076,
                                                        0.00000976, 0.00000884, 0.00000801, 0.00000726, 0.00000657,
                                                        0.00000595, 0.00000538, 0.00000486, 0.00000440, 0.00000397,
                                                        0.00000359, 0.00000324, 0.00000292, 0.00000264, 0.00000238,
                                                        0.00000214, 0.00000193, 0.00000174, 0.00000157, 0.00000141,
                                                        0.00000127, 0.00000114, 0.00000103, 0.00000092, 0.00000083,
                                                        0.00000074, 0.00000067, 0.00000060, 0.00000054, 0.00000048,
                                                        0.00000043, 0.00000039, 0.00000035, 0.00000031, 0.00000028,
                                                        0.00000025, 0.00000022, 0.00000020, 0.00000018, 0.00000016,
                                                        0.00000014, 0.00000013, 0.00000011, 0.00000010, 0.00000009,
                                                        0.00000008, 0.00000007, 0.00000006, 0.00000006, 0.00000005,
                                                        0.00000004, 0.00000004, 0.00000003, 0.00000003, 0.00000003,
                                                        0.00000002};

// Where the above if lhood, this a hard coded/optimized version:
//ratio = 0.99; r_min =0;  	r_max = 3;
//lhood = ratio/(r_max -r_min)  + (1-ratio)*lhood ; hard_coded_log_lhood=log(lhood)
float hard_coded_log_lhood[] = {-1.0614388, -1.0614480, -1.0614757, -1.0615217, -1.0615862, -1.0616689, -1.0617698, -1.0618887, -1.0620256, -1.0621803, -1.0623526, -1.0625423, -1.0627491, -1.0629730, -1.0632135, -1.0634705, -1.0637437, -1.0640327, -1.0643372, -1.0646569, -1.0649914, -1.0653405, -1.0657036, -1.0660804, -1.0664705, -1.0668735, -1.0672889, -1.0677164, -1.0681554, -1.0686054, -1.0690662, -1.0695370, -1.0700176, -1.0705073, -1.0710057, -1.0715124, -1.0720267, -1.0725482, -1.0730764, -1.0736108, -1.0741509, -1.0746962, -1.0752462, -1.0758003, -1.0763581, -1.0769191, -1.0774827, -1.0780486, -1.0786162, -1.0791851, -1.0797547, -1.0803247, -1.0808945, -1.0814638, -1.0820321, -1.0825989, -1.0831639, -1.0837267, -1.0842868, -1.0848439, -1.0853977, -1.0859476, -1.0864935, -1.0870350, -1.0875718, -1.0881035, -1.0886298, -1.0891506, -1.0896655, -1.0901742, -1.0906766, -1.0911723, -1.0916613, -1.0921433, -1.0926181, -1.0930855, -1.0935454, -1.0939976, -1.0944421, -1.0948787, -1.0953073, -1.0957277, -1.0961400, -1.0965441, -1.0969398, -1.0973272, -1.0977063, -1.0980769, -1.0984391, -1.0987930, -1.0991384, -1.0994755, -1.0998042, -1.1001246, -1.1004367, -1.1007407, -1.1010364, -1.1013241, -1.1016038, -1.1018756, -1.1021396, -1.1023958, -1.1026444, -1.1028855, -1.1031191, -1.1033454, -1.1035646, -1.1037767, -1.1039819, -1.1041802, -1.1043719, -1.1045570, -1.1047358, -1.1049082, -1.1050746, -1.1052349, -1.1053894, -1.1055382, -1.1056815, -1.1058193, -1.1059518, -1.1060792, -1.1062016, -1.1063192, -1.1064320, -1.1065402, -1.1066440, -1.1067435, -1.1068389, -1.1069302, -1.1070176, -1.1071012, -1.1071811, -1.1072575, -1.1073306, -1.1074003, -1.1074668, -1.1075303, -1.1075909, -1.1076486, -1.1077036, -1.1077560, -1.1078059, -1.1078533, -1.1078985, -1.1079414, -1.1079821, -1.1080208, -1.1080576, -1.1080925, -1.1081256, -1.1081569, -1.1081867, -1.1082148, -1.1082415, -1.1082667, -1.1082906, -1.1083132, -1.1083345, -1.1083547, -1.1083737, -1.1083917, -1.1084086, -1.1084246, -1.1084397, -1.1084538, -1.1084672, -1.1084798, -1.1084917, -1.1085028, -1.1085133, -1.1085231, -1.1085324, -1.1085411, -1.1085492, -1.1085569, -1.1085640, -1.1085707, -1.1085770, -1.1085829, -1.1085885, -1.1085936, -1.1085985, -1.1086030, -1.1086072, -1.1086111, -1.1086148, -1.1086183, -1.1086215, -1.1086245, -1.1086272, -1.1086298, -1.1086323, -1.1086345, -1.1086366, -1.1086385, -1.1086404, -1.1086420, -1.1086436, -1.1086451, -1.1086464, -1.1086477, -1.1086488, -1.1086499, -1.1086509, -1.1086518, -1.1086527, -1.1086534, -1.1086542, -1.1086549, -1.1086555, -1.1086561, -1.1086566, -1.1086571, -1.1086575, -1.1086580, -1.1086583, -1.1086587, -1.1086590, -1.1086593, -1.1086596, -1.1086599, -1.1086601, -1.1086603, -1.1086605, -1.1086607, -1.1086609, -1.1086610, -1.1086611, -1.1086613, -1.1086614, -1.1086615, -1.1086616, -1.1086617, -1.1086618, -1.1086619, -1.1086619, -1.1086620, -1.1086620, -1.1086621, -1.1086621, -1.1086622, -1.1086622, -1.1086623, -1.1086623, -1.1086623, -1.1086624, -1.1086624, -1.1086624, -1.1086624, -1.1086624, -1.1086625, -1.1086625, -1.1086625, -1.1086625, -1.1086625, -1.1086625, -1.1086625, -1.1086625, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626, -1.1086626};

// Disparity:
// sigma 0.025
float top_lookup[]={15.9577, 15.8165, 15.4003, 14.7308, 13.8422, 12.7779, 11.5877, 10.3231, 9.0345, 7.7674, 6.5604, 5.4433, 4.4368, 3.5527, 2.7947, 2.1596, 1.6395, 1.2227, 0.89578, 0.64471, 0.45584, 0.31662, 0.21604, 0.14482, 0.095364, 0.061691, 0.039205, 0.024476, 0.015011, 0.0090444, 0.0053532, 0.0031126, 0.001778, 0.0009977, 0.00054999, 0.00029784, 0.00015845, 8.2811e-05, 4.2517e-05, 2.1444e-05, 1.0625e-05, 5.1718e-06, 2.473e-06, 1.1617e-06, 5.361e-07, 2.4304e-07, 1.0824e-07, 4.7354e-08, 2.0353e-08, 8.5933e-09, 3.5644e-09, 1.4524e-09, 5.8138e-10, 2.2862e-10, 8.832e-11, 3.3518e-11, 1.2496e-11, 4.5766e-12, 1.6466e-12, 5.8201e-13, 2.0209e-13, 6.8935e-14, 2.31e-14, 7.6043e-15, 2.4592e-15, 7.8127e-16, 2.4383e-16, 7.4758e-17, 2.2517e-17, 6.6624e-18, 1.9366e-18, 5.5299e-19, 1.5512e-19, 4.2749e-20, 1.1573e-20, 3.0778e-21, 8.0413e-22, 2.0639e-22, 5.2038e-23, 1.289e-23, 3.1365e-24, 7.4975e-25, 1.7606e-25, 4.0617e-26, 9.2049e-27, 2.0493e-27, 4.4821e-28, 9.6302e-29, 2.0327e-29, 4.2148e-30, 8.5855e-31, 1.718e-31, 3.3774e-32, 6.5224e-33, 1.2374e-33, 2.3062e-34, 4.2225e-35, 7.5947e-36, 1.3419e-36, 2.3294e-37, 3.9721e-38, 6.6539e-39, 1.095e-39, 1.7703e-40, 2.8115e-41, 4.3864e-42, 6.7231e-43, 1.0123e-43, 1.4973e-44, 2.1758e-45, 3.1059e-46, 4.3555e-47, 6.0003e-48, 8.1205e-49, 1.0796e-49, 1.4101e-50, 1.8092e-51, 2.2804e-52, 2.8237e-53, 3.4349e-54, 4.1047e-55, 4.8186e-56, 5.5571e-57, 6.2958e-58, 7.007e-59, 7.6611e-60, 8.2287e-61, 8.6827e-62, 9.0002e-63, 9.165e-64, 9.1683e-65, 9.01e-66, 8.6984e-67, 8.2497e-68, 7.6862e-69, 7.035e-70, 6.3255e-71, 5.5874e-72, 4.8484e-73, 4.133e-74, 3.4611e-75, 2.8474e-76, 2.3012e-77, 1.827e-78, 1.425e-79, 1.0918e-80, 8.2183e-82, 6.077e-83, 4.4144e-84, 3.1502e-85, 2.2084e-86, 1.5209e-87, 1.029e-88, 6.8387e-90, 4.4651e-91, 2.864e-92, 1.8046e-93, 1.1171e-94, 6.793e-96, 4.058e-97, 2.3815e-98, 1.373e-99, 7.7759e-101, 4.3264e-102, 2.3647e-103, 1.2697e-104, 6.6975e-106, 3.4706e-107, 1.7667e-108, 8.8352e-110, 4.3405e-111, 2.0948e-112, 9.9319e-114, 4.6259e-115, 2.1166e-116, 9.514e-118, 4.2011e-119, 1.8224e-120, 7.7661e-122, 3.2512e-123, 1.3371e-124, 5.402e-126, 2.144e-127, 8.3597e-129, 3.202e-130, 1.2049e-131, 4.4538e-133, 1.6173e-134, 5.7697e-136, 2.022e-137, 6.9614e-139, 2.3544e-140, 7.8227e-142, 2.5533e-143, 8.1871e-145, 2.5789e-146, 7.9803e-148, 2.426e-149, 7.2448e-151, 2.1255e-152, 6.1257e-154, 1.7343e-155, 4.8239e-157, 1.3181e-158, 3.538e-160, 9.3294e-162, 2.4167e-163, 6.1502e-165, 1.5375e-166, 3.7761e-168, 9.1103e-170, 2.1593e-171, 5.0276e-173, 1.15e-174, 2.5841e-176, 5.7042e-178, 1.237e-179, 2.6352e-181, 5.5149e-183, 1.1338e-184, 2.29e-186, 4.5436e-188, 8.8561e-190, 1.6958e-191, 3.1899e-193, 5.8946e-195, 1.0701e-196, 1.9083e-198, 3.3433e-200, 5.7541e-202, 9.7287e-204, 1.6159e-205, 2.6366e-207, 4.2263e-209, 6.6552e-211, 1.0295e-212, 1.5645e-214, 2.3357e-216, 3.4256e-218, 4.9354e-220, 6.9855e-222, 9.7128e-224, 1.3267e-225, 1.7803e-227, 2.3468e-229, 3.039e-231, 3.8662e-233, 4.8318e-235, 5.9321e-237, 7.1548e-239, 8.4773e-241, 9.8673e-243, 1.1283e-244, 1.2674e-246, 1.3986e-248, 1.5162e-250, 1.6147e-252, 1.6893e-254, 1.7362e-256, 1.753e-258, 1.7388e-260, 1.6942e-262, 1.6218e-264, 1.525e-266, 1.4088e-268, 1.2785e-270, 1.1398e-272, 9.9826e-275, 8.5888e-277, 7.2594e-279, 6.0276e-281, 4.9167e-283, 3.9398e-285, 3.1014e-287, 2.3984e-289, 1.8221e-291, 1.3598e-293, 9.9699e-296, 7.1808e-298, 5.0808e-300, 3.5316e-302, 2.4115e-304, 1.6177e-306, 1.066e-308, 6.9011e-311, 4.3889e-313, 2.742e-315, 1.6829e-317, 1.0147e-319, 6.324e-322, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float bottom_lookup[]={0.5, 0.55304, 0.60514, 0.65542, 0.7031, 0.74751, 0.78814, 0.82468, 0.85694, 0.88493, 0.90879, 0.92877, 0.9452, 0.95848, 0.96903, 0.97725, 0.98355, 0.98829, 0.9918, 0.99435, 0.99617, 0.99744, 0.99832, 0.99892, 0.99931, 0.99957, 0.99974, 0.99984, 0.99991, 0.99994, 0.99997, 0.99998, 0.99999, 0.99999, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.99999, 0.99999, 0.99998, 0.99997, 0.99994, 0.99991, 0.99984, 0.99974, 0.99957, 0.99931, 0.99892, 0.99832, 0.99744, 0.99617, 0.99435, 0.9918, 0.98829, 0.98355, 0.97725, 0.96903, 0.95848, 0.9452, 0.92877, 0.90879, 0.88493, 0.85694, 0.82468, 0.78814, 0.74751, 0.7031, 0.65542, 0.60514, 0.55304, 0.5 };

using namespace pcl::simulation;

pcl::simulation::RangeLikelihood::RangeLikelihood (int rows, int cols, int row_height, int col_width, Scene::Ptr scene, int x_offset) :
      scene_(scene), rows_(rows), cols_(cols), row_height_(row_height), col_width_(col_width), x_offset_(x_offset)
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
  camera_fx_ = 576.09757860;
  camera_fy_ = 576.09757860;
  camera_cx_ = 321.06398107;
  camera_cy_ = 242.97676897;

  z_near_ = 0.7;
  z_far_ = 20.0;
  
  which_cost_function_ = 2; // default to commonly used meter based function
  
  // default lhood parameters - these should always be set by the user
  // so might want to add to constructor eventually:
  sigma_ = 0.1;
  floor_proportion_ = 0.9; 
}

pcl::simulation::RangeLikelihood::~RangeLikelihood()
{
  delete [] depth_buffer_;
  delete [] color_buffer_;
}

double
pcl::simulation::RangeLikelihood::sampleNormal (double sigma) {
  typedef boost::normal_distribution<double> Normal;
  Normal dist (0.0, sigma);
  boost::variate_generator<boost::minstd_rand&, Normal> norm (generator, dist);
  return (norm ());
}

void
pcl::simulation::RangeLikelihood::setup_projection_matrix ()
{
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();

  // Prepare scaled simulated camera projection matrix
  float sx = ((float) camera_width_) / col_width_;
  float sy = ((float) camera_height_) / row_height_;
  float width = col_width_;
  float height = row_height_;

  float fx = camera_fx_/sx;
  float fy = camera_fy_/sy;
  float cx = camera_cx_/sx;
  float cy = camera_cy_/sy;
  float m[16];
  float z_nf = (z_near_-z_far_);

  m[0] = 2*fx/width;  m[4] = 0;            m[ 8] = 1.0-(2*cx/width);      m[12] = 0;
  m[1] = 0;           m[5] = 2*fy/height;  m[ 9] = 1.0-(2*cy/height);     m[13] = 0;
  m[2] = 0;           m[6] = 0;            m[10] = (z_far_+z_near_)/z_nf; m[14] = 2.0*z_near_*z_far_/z_nf;
  m[3] = 0;           m[7] = 0;            m[11] = -1;                    m[15] = 0;
  glMultMatrixf (m);
}

void
pcl::simulation::RangeLikelihood::computeLikelihoods(float* reference,
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses, std::vector<float> & scores,
  float* depth_field, bool do_depth_field)
{
  glPushAttrib (GL_ALL_ATTRIB_BITS);
  glEnable (GL_COLOR_MATERIAL);
  glClearColor (0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth (1.0);
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // We do not need the color. Though it doesn't seem to affect the performance.
  //glClear(GL_DEPTH_BUFFER_BIT);
  //glColorMask(false, false, false, false);

  // Setup projection matrix
  setup_projection_matrix ();

  // camera view
  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  glEnable (GL_DEPTH_TEST);
  glDepthMask (GL_TRUE);
  glCullFace (GL_FRONT);
  draw_particles (poses);

  glReadBuffer (GL_BACK);
  glFlush ();
  glReadPixels (x_offset_, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth_buffer_);
  glReadPixels (x_offset_, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, color_buffer_);

  scores.resize (cols_*rows_);

  computeScores (cols_, rows_, col_width_, row_height_,
                  reference, depth_buffer_, scores, depth_field,do_depth_field);

  glPopAttrib ();
}

void
pcl::simulation::RangeLikelihood::apply_camera_transform(const Eigen::Isometry3d & pose)
{
  float T[16];
  Eigen::Matrix4d m = pose.matrix ().inverse ();
  T[0] = m(0,0); T[4] = m(0,1); T[8] = m(0,2); T[12] = m(0,3);
  T[1] = m(1,0); T[5] = m(1,1); T[9] = m(1,2); T[13] = m(1,3);
  T[2] = m(2,0); T[6] = m(2,1); T[10] = m(2,2); T[14] = m(2,3);
  T[3] = m(3,0); T[7] = m(3,1); T[11] = m(3,2); T[15] = m(3,3);
  glMultMatrixf(T);
}

void
pcl::simulation::RangeLikelihood::draw_particles(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses)
{
  int n = 0;
  for (int i=0; i<rows_; ++i)
  {
    for (int j=0; j<cols_; ++j)
    {
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();

      glViewport(x_offset_ + j*col_width_, i*row_height_, col_width_, row_height_);

      // Go from Z-up, X-forward coordinate frame
      // to OpenGL Z-out,Y-up [both Right Handed]
      float T[16];
      T[0] =  0;   T[4] = -1.0; T[8] =  0; T[12] = 0;
      T[1] =  0;   T[5] = 0;    T[9] =  1; T[13] = 0;
      T[2] = -1.0; T[6] = 0;    T[10] = 0; T[14] = 0;
      T[3] =  0;   T[7] = 0;    T[11] = 0; T[15] = 1;
      glMultMatrixf(T);

      // Apply camera transformation
      apply_camera_transform(poses[n++]);

      // Draw the planes in each location:
      scene_->draw();
    }
  }
}


/////////////////////////////////////////////////////////////////
// Below are 4 previously used cost functions:
// 0 original scoring method
float costFunction0(float ref_val,float depth_val){
  return  (sqr(ref_val - depth_val));
}

// 1st working cost function:
// Empirical reverse mapping between depthbuffer and true depth:
// Version 0: [25 aug 2011]
// TRUEDEPTH =  1/(1.33 -(DEPTHBUFFER)*1.29)
//float cost = sqr(ref[col%col_width] - 1/(1.33 -(*depth)*1.29));
// Version 1: [29 aug 2011] Exact version using correct mappings:
float costFunction1(float ref_val,float depth_val){
  float cost = sqr (ref_val - 1/(1.4285 -(depth_val)*1.3788));
  if (ref_val < 0){ // all images pixels with no range
    cost =1;
  }
  if (cost > 10){ // required to lessen the effect of modelpixel with no range (ie holes in the model)
    cost =10;
  }
  return log (cost);
}


// 1st working likelihood function (by far most commonly used)
float costFunction2(float ref_val,float depth_val){
  float min_dist = abs(ref_val - 1/(1.4285 -(depth_val)*1.3788));
  int lup = (int) ceil (min_dist*100); // has resolution of 0.01m
  if (lup > 300){ // implicitly this caps the cost if there is a hole in the model
    lup = 300;
  }
  
  float lhood=1;
  if (isnan(depth_val)){ // pixels with nan depth - for openNI null points
    lhood =1; // log(1) = 0 ---> has no effect
  }else if(ref_val < 0){ // all RGB pixels with no depth - for freenect null points
    lhood =1; // log(1) = 0 ---> has no effect
  }else{
    lhood = normal_sigma0x5_normal1x0_range0to3_step0x01[lup];
    // add a ground floor:
    // increasing this will mean that the likelihood is less peaked
    // but you need more particles to do this...
    // with ~90particles user 0.999, for example in the quad dataset
    // ratio of uniform to	normal
    double ratio = 0.99;//was always 0.99; 
    double r_min =0; // metres
    double r_max = 3; // metres
    lhood = ratio/(r_max -r_min)  + (1-ratio)*lhood ;
  }
  return log(lhood);
}


float costFunction3(float ref_val,float depth_val){
  float log_lhood=0;
  // log(1) = 0 ---> has no effect
  if (ref_val < 0){
    // all images pixels with no range
  }  else if (ref_val > 7){
    // ignore long ranges... for now
  }else{ // working range
    float min_dist = abs (ref_val - 0.7253/(1.0360 - (depth_val)));

    int lup = (int) ceil (min_dist*100); // has resulution of 0.01m
    if (lup > 300)
    { // implicitly this caps the cost if there is a hole in the model
      lup = 300;
    }
    log_lhood= hard_coded_log_lhood[lup];
  }
  return  log_lhood;
}

float costFunction4(float ref_val,float depth_val){
  float disparity_diff = abs( ( -0.7253/ref_val +1.0360 ) -  depth_val );

  int top_lup = (int) ceil (disparity_diff*300); // has resulution of 0.001m
  if (top_lup > 300)
  {
    top_lup =300;
  }
  float top = top_lookup[top_lup];// round( abs(x-mu) *1000+1) );

  // bottom:
  //bottom = bottom_lookup(   round(mu*1000+1));
  int bottom_lup = (int) ceil( (depth_val) *300 ); // has resulution of 0.001m
  if (bottom_lup > 300){
    bottom_lup =300;
  }
  float bottom = bottom_lookup[bottom_lup];// round( abs(x-mu) *1000+1) );

  float proportion = 0.999;
  float lhood = proportion + (1-proportion)*(top/bottom);

  // safety fix thats seems to be required due to opengl ayschronizate
  // ask hordur about this
  if (bottom == 0){
    lhood = proportion;
  }

  if (ref_val< 0){ // all images pixels with no range
    lhood =1; // log(1) = 0 ---> has no effect
  }
  return log(lhood);
}



// TODO: WHEN WE'RE HAPPY THIS SHOULD BE "THE" LIKELIHOOD FUNCTION
// add these global variables into the class
// abd use sigma and floor_proportion directly from class also
using boost::math::normal; // typedef provides default type is double.
normal unit_norm_dist(0,1); // (default mean = zero, and standard deviation = unity)
double costFunction5(double measured_depth,double model_disp, double sigma, double floor_proportion){

  // NEED TO CONVERT MEASURED TO DISPARITY
  double measured_disp =  ( -0.7253/measured_depth +1.0360 );
  
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
    lhood =1; // log(1) = 0 ---> has no effect
  }
 
  
  return log(lhood);
}



void
pcl::simulation::RangeLikelihood::computeScores (int cols, int rows,
    int col_width, int row_height, float* reference, float* depth_buffer,
    std::vector<float> & scores, float* depth_field, bool do_depth_field)
{
  float* depth = depth_buffer;
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
  for (int row = 0; row < rows*row_height; row++)
  {
    float* ref = reference + col_width*(row % row_height);
    // for each column: across each image in a column of model images
    for (int col=0; col<cols*col_width;col++)
    {
      float depth_val = (*depth++); // added jan 2012 - check this is not a breaking fix later mfallon
      float score =0;
      if (which_cost_function_ == 0){
        score = costFunction0(ref[col%col_width],depth_val);
      }else if (which_cost_function_ == 1) { 
	score = costFunction1(ref[col%col_width],depth_val);
      }else if (which_cost_function_ == 2){ 
        score = costFunction2(ref[col%col_width],depth_val);
      }else if(which_cost_function_==3){
	score = costFunction3(ref[col%col_width],depth_val);
      }else if (which_cost_function_ == 4){
        score = costFunction4(ref[col%col_width],depth_val);
      }else if (which_cost_function_ == 5){
	
	//double sigma = 0.025;
	//double floor_proportion_ = 0.999;
        score = costFunction5(ref[col%col_width],depth_val,sigma_,floor_proportion_);	
      }
      scores[row/row_height * cols + col/col_width] += score;
      if (do_depth_field){// do you want the cost image as output?
	  depth_field[row*cols*col_width + col] =  score;
      }
    }
  }
}

void
pcl::simulation::RangeLikelihood::getPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
  bool make_global,
  const Eigen::Isometry3d & pose)
{
  // TODO: check if this works for for rows/cols >1
  // i.e. multiple tiled images
  pc->width    = camera_width_;
  pc->height   = camera_height_;
  pc->is_dense = false;
  pc->points.resize (pc->width*pc->height);

  int points_added = 0;

  float camera_fx_reciprocal_ = 1.0f / camera_fx_;
  float camera_fy_reciprocal_ = 1.0f / camera_fy_;
  float zn = z_near_;
  float zf = z_far_;

  // TODO: support decimation
  // Copied the format of RangeImagePlanar::setDepthImage()
  // Use this as a template for decimation
  for (int y = 0; y < (int)camera_height_; ++y)
  {
    for (int x = 0; x < (int)camera_width_; ++x)
    {
      // Find XYZ from normalized 0->1 mapped disparity
      int idx = points_added; // y*camera_width_ + x;
      float d = depth_buffer_[y*camera_width_ + x] ;
      if (d < 1.0)
      {
        float z = zf*zn/((zf-zn)*(d - zf/(zf-zn)));

        // TODO: add mode to ignore points with no return i.e. depth_buffer_ ==1
        // NB: OpenGL uses a Right Hand system with +X right, +Y up, +Z back out of the screen,
        // The Z-buffer is natively -1 (far) to 1 (near)
        // But in this class we invert this to be 0 (near, 0.7m) and 1 (far, 20m)
        // ... so by negating y we get to a right-hand computer vision system
        // which is also used by PCL and OpenNi
        pc->points[idx].z = z;
        pc->points[idx].x = (x-camera_cx_) * z * (-camera_fx_reciprocal_);
        pc->points[idx].y = (y-camera_cy_) * z * (-camera_fy_reciprocal_);

        unsigned char* rgba_ptr = (unsigned char*)&pc->points[idx].rgba;
        (*rgba_ptr) =  color_buffer_[idx*3+2]; // blue
        (*(rgba_ptr+1)) = color_buffer_[idx*3+1]; // green
        (*(rgba_ptr+2)) = color_buffer_[idx*3];// red
        (*(rgba_ptr+3)) = 0;

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
    Eigen::Matrix4f m =  pose.matrix ().cast<float> () * T;
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
    Eigen::Matrix4f camera = pose.matrix ().cast<float> () * cam_to_body;
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
  float variance = 0.0015;
  for (int i = 0; i < camera_width_*camera_height_ ; i++)
  {
    if (depth_buffer_[i] < 1)
    {
      depth_buffer_[i] = depth_buffer_[i] + variance*sampleNormal ();
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
  // The fitted model here cannot work for long ranges:
  // http://www.ros.org/wiki/kinect_calibration/technical
  // TODO: make a parameter
  int bins = 470;
  for (int i = 0; i < camera_width_*camera_height_ ; i++)
  {
    depth_buffer_[i] =  float (int (depth_buffer_[i]*bins + 0.5))/bins;
  }
  cout << "in add noise\n";
}

// Finds the maximum level n so a and b are still
// divisible by 2^n
int
max_level(int a, int b)
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
void display_tic_toc(vector<double> &tic_toc,const string &fun_name){
  int tic_toc_size = tic_toc.size();
  
  double percent_tic_toc_last = 0;
  double dtime = ((double) (tic_toc[tic_toc_size-1] - tic_toc[0]));
  cout << "fraction_" << fun_name << ",";  
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";  
  
}



// Shader based rangelikelihood
pcl::simulation::RangeLikelihoodGLSL::RangeLikelihoodGLSL(int rows,
                                                          int cols,
                                                          int row_height,
                                                          int col_width,
                                                          Scene::Ptr scene,
                                                          int x_offset) : pcl::simulation::RangeLikelihood (rows, cols, row_height, col_width, scene, x_offset),
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
  
  int height = rows * row_height;
  int width = cols * col_width;

  // For now we only support a limited size texture
  assert(height >0 && height <= 8192 && width > 0 && width <= 8192 );
  // throw std::runtime_error "

  // Allocate framebuffer
  glGenRenderbuffers (1, &depth_render_buffer_);
  glBindRenderbuffer (GL_RENDERBUFFER, depth_render_buffer_);
  glRenderbufferStorage (GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width, height);

  glGenRenderbuffers (1, &color_render_buffer_);
  glBindRenderbuffer (GL_RENDERBUFFER, color_render_buffer_);
  glRenderbufferStorage (GL_RENDERBUFFER, GL_RGB8UI, width, height);

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

  if (gllib::get_gl_error() != GL_NO_ERROR)
  {
    std::cerr << "RangeLikelihoodGLSL::RangeLikelihoodGLSL: Failed initializing OpenGL buffers" << std::endl;
    exit(-1);
  }

  glGenFramebuffers (1, &score_fbo_);
  glBindFramebuffer (GL_FRAMEBUFFER, score_fbo_);
  glFramebufferRenderbuffer (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, score_texture_, 0);

  // Load shader
  likelihood_program_ = gllib::Program::Ptr (new gllib::Program ());
  // TODO: to remove file dependency include the shader source in the binary
  if (!likelihood_program_->add_shader_file ("compute_score.vert", gllib::VERTEX))
  {
    std::cout << "Failed loading vertex shader" << std::endl;
    exit (-1);
  }
  
  if (!likelihood_program_->add_shader_file ("compute_score.frag", gllib::FRAGMENT))
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

  gllib::get_gl_error ();

  // Go back to the default pipeline
  glUseProgram (0);

  score_buffer_ = new float[width_*height_];
}

pcl::simulation::RangeLikelihoodGLSL::~RangeLikelihoodGLSL ()
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

  delete [] score_buffer_;
}

void
RangeLikelihoodGLSL::computeLikelihoods (float* reference,
                     std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses,
                     std::vector<float> & scores,
                     float *depth_field,
                     bool do_depth_field )
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
    tic_toc.push_back(getTime());
  #endif  
  

  #if DO_TIMING_PROFILE
    tic_toc.push_back(getTime());
  #endif  
  // The depth image is now in depth_texture_

  // Compute likelihoods
  if (compute_likelihood_on_cpu_)
  {
    computeScores (cols_, rows_,
                   col_width_, row_height_,
                   reference, depth_buffer_,
                   scores, depth_field, do_depth_field);
  }
  else
  {
    computeScoresShader (cols_, rows_,
                         col_width_, row_height_,
                         reference, depth_buffer_,
                         scores, depth_field, do_depth_field);
    
    //#if DO_TIMING_PROFILE
    //  tic_toc.push_back(getTime());
    //#endif  


    // The scores are in score_texture_
    bool get_score_texture = true;
    if (aggregate_on_cpu_ || get_score_texture)
    {
      glActiveTexture (GL_TEXTURE0);
      glBindTexture (GL_TEXTURE_2D, score_texture_);
      glGetTexImage (GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, score_buffer_);
      glBindTexture (GL_TEXTURE_2D, 0);
    }

    // Aggregate results (we do not use GPU to sum cpu scores)
    if (aggregate_on_cpu_)
    {
      glActiveTexture (GL_TEXTURE0);
      glBindTexture (GL_TEXTURE_2D, score_texture_);
      glGetTexImage (GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, score_buffer_);
      glBindTexture (GL_TEXTURE_2D, 0);

      for (int n = 0, row = 0; row < height_; ++row)
      {
        for (int col = 0; col < width_; ++col, ++n)
        {
          scores[row/row_height_ * cols_ + col/col_width_] += score_buffer_[n];
        }
      }
      glActiveTexture (GL_TEXTURE0);
      glBindTexture (GL_TEXTURE_2D, 0);
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
    tic_toc.push_back(getTime());
    display_tic_toc(tic_toc,"range_likelihood");
  #endif    
}

// Computes the likelihood scores using a shader
void
pcl::simulation::RangeLikelihoodGLSL::computeScoresShader(int cols,
                                                            int rows,
                                                            int col_width,
                                                            int row_height,
                                                            float* reference,
                                                            float* depth_buffer,
                                                            std::vector<float> & scores,
                                                            float* depth_field,
                                                            bool do_depth_field)
{
  if (gllib::get_gl_error () != GL_NO_ERROR)
  {
    std::cout << "GL error: RangeLikelihoodGLSL::compute_scores_shader - enter" << std::endl;
  }

#ifdef SIMULATION_DEBUG
  std::cout << "DepthSampler location: " << likelihood_program_->get_uniform_location ("DepthSampler") << std::endl;
  std::cout << "ReferenceSampler location: " << likelihood_program_->get_uniform_location ("ReferenceSampler") << std::endl;
  std::cout << "CostSampler location: " << likelihood_program_->get_uniform_location ("CostSampler") << std::endl;

  int depth_tex_id;
  int ref_tex_id;
  int cost_tex_id;

  glGetUniformiv(likelihood_program_->program_id (), likelihood_program_->get_uniform_location ("DepthSampler"), &depth_tex_id);
  glGetUniformiv(likelihood_program_->program_id (), likelihood_program_->get_uniform_location ("ReferenceSampler"), &ref_tex_id);
  glGetUniformiv(likelihood_program_->program_id (), likelihood_program_->get_uniform_location ("CostSampler"), &cost_tex_id);
  std::cout << "depth id: " << depth_tex_id << " " << GL_TEXTURE0 << std::endl;
  std::cout << "ref id: " << ref_tex_id  << " " << GL_TEXTURE1 << std::endl;
  std::cout << "cost id: " << cost_tex_id << " " << GL_TEXTURE2 << std::endl;
#endif

  likelihood_program_->use ();
  likelihood_program_->set_uniform ("DepthSampler", 0);
  likelihood_program_->set_uniform ("ReferenceSampler", 1);
  likelihood_program_->set_uniform ("CostSampler", 2);
  likelihood_program_->set_uniform ("cols", cols_);
  likelihood_program_->set_uniform ("rows", rows_);
  likelihood_program_->set_uniform ("near", z_near_);
  likelihood_program_->set_uniform ("far", z_far_);

  glBindFramebuffer (GL_FRAMEBUFFER, score_fbo_);
  glDrawBuffer (GL_COLOR_ATTACHMENT0);
  glReadBuffer (GL_NONE);
  glDisable (GL_DEPTH_TEST);
  glViewport (0, 0, width_, height_);

  // Setup textures
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, depth_texture_);

  glActiveTexture (GL_TEXTURE1);
  glBindTexture (GL_TEXTURE_2D, sensor_texture_);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_R32F, col_width, row_height, 0, GL_RED, GL_FLOAT, reference);

  glActiveTexture (GL_TEXTURE2);
  glBindTexture (GL_TEXTURE_2D, likelihood_texture_);

  quad_.render ();
  glUseProgram (0);

  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  // Unbind all texture that were used
  glActiveTexture (GL_TEXTURE0);
  glBindTexture (GL_TEXTURE_2D, 0);
  glActiveTexture (GL_TEXTURE1);
  glBindTexture (GL_TEXTURE_2D, 0);
  glActiveTexture (GL_TEXTURE2);
  glBindTexture (GL_TEXTURE_2D, 0);
}

void
RangeLikelihoodGLSL::render (const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > & poses)
{
  if (gllib::get_gl_error () != GL_NO_ERROR)
  {
    std::cerr << "GL Error: RangeLikelihoodGLSL::render" << std::endl;
  }

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
      std::cout << "Framebuffer failed" << std::endl;
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
  setup_projection_matrix ();

  glEnable (GL_DEPTH_TEST);
  glDepthMask (GL_TRUE);
  glCullFace (GL_FRONT);
  draw_particles (poses);

  glPopAttrib ();

  glFlush ();
  
  // Read depth
  glReadPixels (0, 0, width_, height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth_buffer_);

  // Read Color
  glReadBuffer (GL_COLOR_ATTACHMENT0);
  glReadPixels (0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, color_buffer_);
  glBindFramebuffer (GL_FRAMEBUFFER, 0);  
}
