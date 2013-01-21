
points = zeros(3,10);
camera = zeros(3,4);
camera(1:3,1:3) = eye(3);

% alpha = 90;
% camera(1,1) = 1;
% camera(2,2) = cos(alpha);
% camera(3,3) = cos(alpha);
% camera(2,3) = sin(alpha);
% camera(3,2) = -sin(alpha);

pixelnoise = 0;


PoseCovariance = matlabTrackerIF(pixelnoise, points, camera);
