% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1759.958304469675600 ; 1758.223895262386800 ];

%-- Principal point:
cc = [ 963.800293674975480 ; 541.048105060508990 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.162616653115618 ; -0.674453661979077 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 5.220725005243889 ; 5.303246937731103 ];

%-- Principal point uncertainty:
cc_error = [ 2.086964713837343 ; 2.537527307873333 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.008911232502239 ; 0.042599834271441 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Image size:
nx = 1920;
ny = 1080;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 10;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 0 ; 0 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.123352e+00 ; 2.072622e+00 ; -8.656565e-02 ];
Tc_1  = [ -4.447190e-02 ; -2.911925e-02 ; 1.313502e-01 ];
omc_error_1 = [ 1.269484e-03 ; 1.373888e-03 ; 2.488629e-03 ];
Tc_error_1  = [ 1.555453e-04 ; 1.846236e-04 ; 4.061057e-04 ];

%-- Image #2:
omc_2 = [ 2.259159e+00 ; 1.596236e+00 ; -6.433324e-02 ];
Tc_2  = [ -5.068115e-02 ; -1.530444e-02 ; 1.577072e-01 ];
omc_error_2 = [ 1.638816e-03 ; 1.457673e-03 ; 2.559259e-03 ];
Tc_error_2  = [ 1.898654e-04 ; 2.245881e-04 ; 4.867189e-04 ];

%-- Image #3:
omc_3 = [ 1.697776e+00 ; 2.333626e+00 ; -4.201125e-01 ];
Tc_3  = [ -4.114309e-02 ; -3.865066e-02 ; 1.631479e-01 ];
omc_error_3 = [ 1.183764e-03 ; 1.752038e-03 ; 2.466129e-03 ];
Tc_error_3  = [ 1.839354e-04 ; 2.276527e-04 ; 4.721079e-04 ];

%-- Image #4:
omc_4 = [ 1.833404e+00 ; 1.828624e+00 ; 2.658768e-01 ];
Tc_4  = [ -3.874103e-02 ; -3.117591e-02 ; 1.264090e-01 ];
omc_error_4 = [ 1.444597e-03 ; 1.296973e-03 ; 1.741390e-03 ];
Tc_error_4  = [ 1.581827e-04 ; 1.788781e-04 ; 4.068253e-04 ];

%-- Image #5:
omc_5 = [ 2.013213e+00 ; 1.428063e+00 ; 3.838377e-01 ];
Tc_5  = [ -4.270374e-02 ; -2.129342e-02 ; 1.443120e-01 ];
omc_error_5 = [ 1.654153e-03 ; 1.151800e-03 ; 1.843662e-03 ];
Tc_error_5  = [ 1.804601e-04 ; 2.053265e-04 ; 4.797034e-04 ];

%-- Image #6:
omc_6 = [ -2.168853e+00 ; -2.188238e+00 ; -3.536689e-01 ];
Tc_6  = [ -3.969744e-02 ; -2.842907e-02 ; 1.191465e-01 ];
omc_error_6 = [ 1.338739e-03 ; 1.379334e-03 ; 2.574391e-03 ];
Tc_error_6  = [ 1.475932e-04 ; 1.762561e-04 ; 3.734792e-04 ];

%-- Image #7:
omc_7 = [ 1.927827e+00 ; 1.693234e+00 ; -5.487722e-01 ];
Tc_7  = [ -4.785870e-02 ; -1.651949e-02 ; 1.467780e-01 ];
omc_error_7 = [ 1.199942e-03 ; 1.421322e-03 ; 1.624402e-03 ];
Tc_error_7  = [ 1.652099e-04 ; 2.100979e-04 ; 3.942223e-04 ];

%-- Image #8:
omc_8 = [ 2.057132e+00 ; 1.450318e+00 ; -7.444185e-02 ];
Tc_8  = [ -4.104870e-02 ; -2.054376e-02 ; 1.142954e-01 ];
omc_error_8 = [ 1.363382e-03 ; 1.181188e-03 ; 1.351931e-03 ];
Tc_error_8  = [ 1.375358e-04 ; 1.607556e-04 ; 3.398713e-04 ];

%-- Image #9:
omc_9 = [ 2.119097e+00 ; 2.085730e+00 ; -5.611119e-02 ];
Tc_9  = [ -4.298302e-02 ; -2.650258e-02 ; 1.141300e-01 ];
omc_error_9 = [ 1.108608e-03 ; 1.163131e-03 ; 2.074669e-03 ];
Tc_error_9  = [ 1.357376e-04 ; 1.607885e-04 ; 3.512150e-04 ];

%-- Image #10:
omc_10 = [ 2.093106e+00 ; 2.012592e+00 ; 4.728829e-02 ];
Tc_10  = [ -4.180126e-02 ; -2.670669e-02 ; 1.193557e-01 ];
omc_error_10 = [ 1.227097e-03 ; 1.240230e-03 ; 2.134248e-03 ];
Tc_error_10  = [ 1.460508e-04 ; 1.689384e-04 ; 3.729845e-04 ];

