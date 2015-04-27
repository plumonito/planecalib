% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 756.290023668920640 ; 756.754084938735790 ];

%-- Principal point:
cc = [ 324.307778064432910 ; 252.104524589890080 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.389839386716848 ; 0.197068948715649 ; -0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.633269239810729 ; 1.661350476899725 ];

%-- Principal point uncertainty:
cc_error = [ 3.425833615508290 ; 2.624450043925099 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013383411694800 ; 0.062332293611207 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 16;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 0 ; 0 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.177573e+00 ; 1.948284e+00 ; -4.194480e-01 ];
Tc_1  = [ -4.618970e+02 ; -2.357158e+02 ; 1.211833e+03 ];
omc_error_1 = [ 3.270674e-03 ; 4.224043e-03 ; 7.472601e-03 ];
Tc_error_1  = [ 5.589706e+00 ; 4.374136e+00 ; 3.616442e+00 ];

%-- Image #2:
omc_2 = [ -1.940417e+00 ; -2.142211e+00 ; 1.093138e+00 ];
Tc_2  = [ -4.211376e+02 ; -2.662639e+02 ; 1.377700e+03 ];
omc_error_2 = [ 5.125948e-03 ; 2.908042e-03 ; 6.247680e-03 ];
Tc_error_2  = [ 6.408186e+00 ; 5.002793e+00 ; 3.294359e+00 ];

%-- Image #3:
omc_3 = [ 1.730374e+00 ; 1.947014e+00 ; -8.158781e-01 ];
Tc_3  = [ -3.449061e+02 ; -2.652062e+02 ; 1.347044e+03 ];
omc_error_3 = [ 2.450954e-03 ; 4.355108e-03 ; 6.023638e-03 ];
Tc_error_3  = [ 6.206104e+00 ; 4.793102e+00 ; 3.180483e+00 ];

%-- Image #4:
omc_4 = [ 1.544440e+00 ; 1.990685e+00 ; -1.404869e+00 ];
Tc_4  = [ -1.249660e+02 ; -2.139579e+02 ; 1.610911e+03 ];
omc_error_4 = [ 3.035500e-03 ; 4.600611e-03 ; 5.692604e-03 ];
Tc_error_4  = [ 7.400362e+00 ; 5.632909e+00 ; 2.959440e+00 ];

%-- Image #5:
omc_5 = [ 1.744019e+00 ; 1.583465e+00 ; -5.981900e-01 ];
Tc_5  = [ -5.612741e+02 ; -6.085725e+01 ; 1.585804e+03 ];
omc_error_5 = [ 2.687119e-03 ; 4.190857e-03 ; 5.549611e-03 ];
Tc_error_5  = [ 7.219050e+00 ; 5.717273e+00 ; 4.387511e+00 ];

%-- Image #6:
omc_6 = [ 1.564150e+00 ; 1.745483e+00 ; -2.773302e-01 ];
Tc_6  = [ -2.586797e+02 ; -4.123182e+02 ; 1.526555e+03 ];
omc_error_6 = [ 2.968315e-03 ; 4.405323e-03 ; 5.536681e-03 ];
Tc_error_6  = [ 7.035689e+00 ; 5.384489e+00 ; 4.180712e+00 ];

%-- Image #7:
omc_7 = [ 6.431912e-01 ; 2.390189e+00 ; -6.308601e-01 ];
Tc_7  = [ -6.803264e+01 ; -4.614052e+02 ; 1.840383e+03 ];
omc_error_7 = [ 2.172638e-03 ; 5.151608e-03 ; 5.641708e-03 ];
Tc_error_7  = [ 8.491822e+00 ; 6.489867e+00 ; 4.219913e+00 ];

%-- Image #8:
omc_8 = [ 2.459633e+00 ; 9.461031e-01 ; -1.160833e+00 ];
Tc_8  = [ -2.894528e+02 ; 8.830762e+01 ; 1.794264e+03 ];
omc_error_8 = [ 3.886363e-03 ; 3.487615e-03 ; 6.717668e-03 ];
Tc_error_8  = [ 8.173813e+00 ; 6.251309e+00 ; 3.738245e+00 ];

%-- Image #9:
omc_9 = [ 1.959787e+00 ; 1.923899e+00 ; -5.319321e-01 ];
Tc_9  = [ -4.209250e+02 ; -2.685427e+02 ; 1.272433e+03 ];
omc_error_9 = [ 2.786358e-03 ; 4.280483e-03 ; 6.700984e-03 ];
Tc_error_9  = [ 5.866274e+00 ; 4.569125e+00 ; 3.529187e+00 ];

%-- Image #10:
omc_10 = [ 1.989654e+00 ; 2.035048e+00 ; -8.458489e-01 ];
Tc_10  = [ -3.038227e+02 ; -2.420215e+02 ; 1.457358e+03 ];
omc_error_10 = [ 2.801896e-03 ; 4.274500e-03 ; 6.864219e-03 ];
Tc_error_10  = [ 6.705317e+00 ; 5.121127e+00 ; 3.218327e+00 ];

%-- Image #11:
omc_11 = [ 2.176066e+00 ; 2.194694e+00 ; -5.370290e-01 ];
Tc_11  = [ -2.496998e+02 ; -2.784615e+02 ; 1.511191e+03 ];
omc_error_11 = [ 3.959419e-03 ; 4.532290e-03 ; 8.854391e-03 ];
Tc_error_11  = [ 6.953737e+00 ; 5.265195e+00 ; 3.569145e+00 ];

%-- Image #12:
omc_12 = [ 1.684881e+00 ; 1.663031e+00 ; 3.201047e-01 ];
Tc_12  = [ -2.642815e+02 ; -2.789502e+02 ; 8.826655e+02 ];
omc_error_12 = [ 3.384563e-03 ; 3.533252e-03 ; 5.066513e-03 ];
Tc_error_12  = [ 4.100522e+00 ; 3.134969e+00 ; 2.854470e+00 ];

%-- Image #13:
omc_13 = [ 1.571001e+00 ; 1.513374e+00 ; 4.093590e-01 ];
Tc_13  = [ -2.899704e+02 ; -2.706202e+02 ; 8.165474e+02 ];
omc_error_13 = [ 3.342250e-03 ; 3.523195e-03 ; 4.487218e-03 ];
Tc_error_13  = [ 3.833879e+00 ; 2.928371e+00 ; 2.752491e+00 ];

%-- Image #14:
omc_14 = [ -1.552926e+00 ; -1.922907e+00 ; 1.118852e+00 ];
Tc_14  = [ -3.047080e+02 ; -3.280215e+02 ; 1.542529e+03 ];
omc_error_14 = [ 4.655036e-03 ; 3.400884e-03 ; 5.142567e-03 ];
Tc_error_14  = [ 7.157299e+00 ; 5.494224e+00 ; 3.337501e+00 ];

%-- Image #15:
omc_15 = [ 1.317103e+00 ; 1.442141e+00 ; -1.169018e+00 ];
Tc_15  = [ -3.014819e+02 ; 9.165943e+01 ; 1.494875e+03 ];
omc_error_15 = [ 3.064978e-03 ; 4.216116e-03 ; 4.353190e-03 ];
Tc_error_15  = [ 6.829133e+00 ; 5.236216e+00 ; 2.845252e+00 ];

%-- Image #16:
omc_16 = [ 1.314178e+00 ; 1.440339e+00 ; -1.173105e+00 ];
Tc_16  = [ -3.009007e+02 ; 1.041928e+02 ; 1.495373e+03 ];
omc_error_16 = [ 3.063125e-03 ; 4.207764e-03 ; 4.345262e-03 ];
Tc_error_16  = [ 6.834400e+00 ; 5.240305e+00 ; 2.841779e+00 ];

