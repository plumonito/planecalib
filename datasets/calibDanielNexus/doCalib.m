numImages = 10;
files = cell(1, numImages);
for i = 1:numImages
    files{i} = fullfile(pwd, sprintf('calib%.4d.jpg', i));
end

% Display one of the calibration images
magnification = 25;
figure; imshow(files{1}, 'InitialMagnification', magnification);
title('One of the Calibration Images');

% Detect the checkerboard corners in the images.
[imageCorners, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 29; % in millimeters
worldCorners = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
cameraParameters = estimateCameraParameters(imageCorners, worldCorners);

% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParameters);