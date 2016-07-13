function q = inverse_kinematics(foot_right,foot_left)
% This function computes the inverse kinematics for the March exoskeleton
% using a closed form solution of the robot kinematics.

% q = inverse_kinematics(foot_right,foot_left)
%
% inputs 'foot_right' and 'foot_left' are n by 3 vectors respresenting n 
% task space positions with coordinates x,y,z of the feet with respect to 
% the torso
%
% output 'q' is an n by 6 vector representing n joint coordinates:
% [ Rx_right_hip
%   Ry_right_hip
%   Ry_right_knee
%   Rx_left_hip
%   Ry_left_hip
%   Ry_left_knee ]

%% Initialize parameters
param
n = size(foot_right,1);

%% Solve inverse kinematics for right leg
% compute right hip location with repect to right foot
torso_right = repmat([0 -torso_width 0],n,1); %right hip location with respect to torso
torso_foot = torso_right-foot_right; %torso position with respect to right foot

% compute inverse kinematics for right hip AA using goniometric relations
q(:,1) = pi/2-atan(torso_foot(:,2)./torso_foot(:,3))-atan(sqrt(torso_foot(:,2).^2+torso_foot(:,3).^2-pelvis_width.^2)./pelvis_width);

% compute position of right hip
pelvis_right = torso_right+[zeros(n,1) -cos(q(:,1)).*pelvis_width -sin(q(:,1)).*pelvis_width]; %right pelvis position with respect to torso
hip_right = pelvis_right+repmat([pelvis_depth 0 0],n,1); %right hip position with respect to torso
hip_foot = hip_right-foot_right; %right hip position with respect to right foot

% compute distance between right foot and right hip
right_leg_length = sqrt(hip_foot(:,1).^2+hip_foot(:,2).^2+hip_foot(:,3).^2);

% compute inverse kinematics for right hip FE and knee FE using goniometric relations
q(:,2) = pi/2-acos(hip_foot(:,1)./right_leg_length)-acos((lower_length^2-upper_length^2-right_leg_length.^2)./(-2*upper_length*right_leg_length));
q(:,3) = pi-acos((right_leg_length.^2-upper_length^2-lower_length^2)./(-2*upper_length*lower_length));

%% Solve inverse kinematics for left leg
% compute left hip location with repect to left foot
torso_left = repmat([0 torso_width 0],n,1); %left hip location with respect to torso
torso_foot = torso_left-foot_left; %torso position with respect to left foot

% compute inverse kinematics for left hip AA using goniometric relations
q(:,4) = -pi/2+atan(-torso_foot(:,2)./torso_foot(:,3))+atan(sqrt(torso_foot(:,2).^2+torso_foot(:,3).^2-pelvis_width.^2)./pelvis_width);

% compute position of left hip
pelvis_left = torso_left+[zeros(n,1) cos(q(:,4)).*pelvis_width sin(q(:,4)).*pelvis_width]; %left pelvis position with respect to torso
hip_left = pelvis_left+repmat([pelvis_depth 0 0],n,1); %left hip position with respect to torso
hip_foot = hip_left-foot_left; %left hip position with respect to left foot

% compute distance between left foot and left hip
left_leg_length = sqrt(hip_foot(:,1).^2+hip_foot(:,2).^2+hip_foot(:,3).^2);

% compute inverse kinematics for left hip FE and knee FE using goniometric relations
q(:,5) = pi/2-acos(hip_foot(:,1)./left_leg_length)-acos((lower_length^2-upper_length^2-left_leg_length.^2)./(-2*upper_length*left_leg_length));
q(:,6) = pi-acos((left_leg_length.^2-upper_length^2-lower_length^2)./(-2*upper_length*lower_length));