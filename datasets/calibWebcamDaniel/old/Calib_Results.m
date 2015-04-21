% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 621.393815825299500 ; 620.538649108112740 ];

%-- Principal point:
cc = [ 314.140073205802650 ; 238.535942967520360 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.000000000000000 ; -0.000000000000000 ; -0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 9.177719746676502 ; 8.698562569564613 ];

%-- Principal point uncertainty:
cc_error = [ 1.578485479999289 ; 4.104097693692754 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 12;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 0 ; 0 ; 0 ; 0 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.122249e+00 ; 2.124004e+00 ; -1.555554e-01 ];
Tc_1  = [ -4.773022e+02 ; -3.529024e+02 ; 1.202245e+03 ];
omc_error_1 = [ 3.113050e-03 ; 3.201902e-03 ; 5.888914e-03 ];
Tc_error_1  = [ 3.003351e+00 ; 7.685947e+00 ; 1.773511e+01 ];

%-- Image #2:
omc_2 = [ -2.159613e+00 ; -2.220642e+00 ; 4.226400e-01 ];
Tc_2  = [ -4.965988e+02 ; -3.451455e+02 ; 1.250446e+03 ];
omc_error_2 = [ 2.347528e-03 ; 2.670339e-03 ; 8.391828e-03 ];
Tc_error_2  = [ 2.994750e+00 ; 7.975905e+00 ; 1.814590e+01 ];

%-- Image #3:
omc_3 = [ 2.060672e+00 ; 2.015186e+00 ; 2.743320e-02 ];
Tc_3  = [ -4.297606e+02 ; -3.760354e+02 ; 1.066772e+03 ];
omc_error_3 = [ 4.160437e-03 ; 3.325005e-03 ; 4.548803e-03 ];
Tc_error_3  = [ 2.874367e+00 ; 6.842376e+00 ; 1.555949e+01 ];

%-- Image #4:
omc_4 = [ 1.956580e+00 ; 2.016158e+00 ; -3.903949e-01 ];
Tc_4  = [ -4.674718e+02 ; -3.175771e+02 ; 1.262288e+03 ];
omc_error_4 = [ 3.852613e-03 ; 3.877462e-03 ; 6.238208e-03 ];
Tc_error_4  = [ 2.996929e+00 ; 8.116683e+00 ; 1.825580e+01 ];

%-- Image #5:
omc_5 = [ 2.181240e+00 ; 2.203045e+00 ; -1.040256e-01 ];
Tc_5  = [ -4.815737e+02 ; -3.128583e+02 ; 1.192635e+03 ];
omc_error_5 = [ 3.076971e-03 ; 3.088720e-03 ; 6.372250e-03 ];
Tc_error_5  = [ 2.990975e+00 ; 7.753163e+00 ; 1.757234e+01 ];

%-- Image #6:
omc_6 = [ 2.183225e+00 ; 2.215941e+00 ; -8.609221e-02 ];
Tc_6  = [ -4.799394e+02 ; -3.127745e+02 ; 1.081365e+03 ];
omc_error_6 = [ 2.577095e-03 ; 2.581814e-03 ; 5.320827e-03 ];
Tc_error_6  = [ 2.717550e+00 ; 7.041781e+00 ; 1.589939e+01 ];

%-- Image #7:
omc_7 = [ 1.817621e+00 ; 2.392399e+00 ; -1.443623e-01 ];
Tc_7  = [ -3.536047e+02 ; -4.609567e+02 ; 1.335432e+03 ];
omc_error_7 = [ 3.338218e-03 ; 4.183431e-03 ; 6.699423e-03 ];
Tc_error_7  = [ 3.351771e+00 ; 8.501600e+00 ; 1.965766e+01 ];

%-- Image #8:
omc_8 = [ 2.242690e+00 ; 1.883297e+00 ; -2.457323e-01 ];
Tc_8  = [ -5.174440e+02 ; -2.274201e+02 ; 1.428274e+03 ];
omc_error_8 = [ 4.291209e-03 ; 3.885555e-03 ; 7.627619e-03 ];
Tc_error_8  = [ 3.531062e+00 ; 9.256427e+00 ; 2.114384e+01 ];

%-- Image #9:
omc_9 = [ 2.134254e+00 ; 1.849498e+00 ; 3.019527e-02 ];
Tc_9  = [ -4.345928e+02 ; -2.710671e+02 ; 1.229504e+03 ];
omc_error_9 = [ 4.983448e-03 ; 3.640529e-03 ; 5.637411e-03 ];
Tc_error_9  = [ 3.288686e+00 ; 7.936868e+00 ; 1.810243e+01 ];

%-- Image #10:
omc_10 = [ -2.052091e+00 ; -2.191304e+00 ; 5.964854e-01 ];
Tc_10  = [ -3.886662e+02 ; -3.560678e+02 ; 1.547632e+03 ];
omc_error_10 = [ 2.965425e-03 ; 3.651273e-03 ; 1.103816e-02 ];
Tc_error_10  = [ 3.909299e+00 ; 1.004276e+01 ; 2.135003e+01 ];

%-- Image #11:
omc_11 = [ -2.026006e+00 ; -2.169235e+00 ; 5.522004e-01 ];
Tc_11  = [ -4.513719e+02 ; -3.435186e+02 ; 1.367373e+03 ];
omc_error_11 = [ 2.473346e-03 ; 3.468951e-03 ; 9.904338e-03 ];
Tc_error_11  = [ 3.442188e+00 ; 8.789621e+00 ; 1.889670e+01 ];

%-- Image #12:
omc_12 = [ 2.099744e+00 ; 2.174401e+00 ; -2.337878e-01 ];
Tc_12  = [ -4.672572e+02 ; -3.657933e+02 ; 1.228966e+03 ];
omc_error_12 = [ 2.954895e-03 ; 3.156018e-03 ; 6.289704e-03 ];
Tc_error_12  = [ 3.001687e+00 ; 7.821317e+00 ; 1.816309e+01 ];

