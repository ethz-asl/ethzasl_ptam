clear all;

npoints = 200;

distrib_plane = 3;
distrib_plane2 = 15;

zplane = 0;
pixelnoise = 0.5;
stripewidth = 0.1;
stripe_location = 15;


explored_area_core = randn(3, npoints) * distrib_plane;
explored_area_core(2,:) = explored_area_core(2,:)*stripewidth - 15;

explored_area_stripe = randn(3, npoints) * distrib_plane;
explored_area_stripe(2,:) = explored_area_stripe(2,:) + stripe_location;

unexplored_area = randn(3, npoints) * distrib_plane2;


points = [explored_area_core explored_area_stripe unexplored_area];

points(3,:) = zplane + points(3,:) * 0.001;

% points(:,1) =[-0.520898 0.059774 -0.00237556];
% points(:,2) =[ -0.536628 0.0557534 0.0170862];
% points(:,3) =[ -0.926411 -0.103911 -0.0103726];
% points(:,4) =[ -0.913931 -0.127581 0.0193194];
% points(:,5) =[ -0.90015 -0.122935 -0.0160205];
% points(:,6) =[ -0.71537 -0.107164 -0.00494531];
% points(:,7) =[ -0.801244 0.176297 -0.00502234];
% points(:,8) =[ -0.814134 0.169979 0.0189816];
% points(:,9) =[ -0.429025 0.0571335 -0.000828381];
% points(:,10) =[ -0.62596 0.137993 0.0405261];
%
% camera = [0.991464 0.124893 -0.0374202 0.789354
% 0.117282 -0.97972 -0.162461 0.0709251
% -0.0569516 0.156685 -0.986005 1.64471];

camera = zeros(3,4);
camera(1:3,1:3) = eye(3);
%rotation
camera(1,1) = 1;
camera(2,2) = -1;
camera(3,3) = -1;
camera(2,3) = 0;
camera(3,2) = 0;


sz = distrib_plane2*3;
steps = 15;

res = zeros(steps);

x = [-sz:sz/steps:sz];

for i = 1:length(x)
    for j = 1:length(x)
        camera(:,4) = [x(i) x(j) 8]';
        PoseCovarianceInv = matlabTrackerIF(pixelnoise, points, camera);
        if sum(isnan(PoseCovarianceInv)) == 0
            PoseCovariance = pinv(PoseCovarianceInv);
            res(i,j) = trace(PoseCovariance);
        else
            res(i,j) = NaN;
        end
    end
end


surf(x, x, res);
hold on;
plot3(points(2,:), -points(1,:),points(3,:), 'xr');
hold off;
