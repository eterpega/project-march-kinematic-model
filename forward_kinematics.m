function [ankle_right,ankle_left,knee_right,knee_left,hip_right,hip_left,pelvis_right,pelvis_left,torso_right,torso_left] = forward_kinematics(q)
% This function computes the forward kinematics for the March exoskeleton.
%
% [foot_right,foot_left] = forward_kinematics(q)
% 
% input 'q' is an n by 6 vector representing n joint coordinates:
% [ Rx_right_hip
%   Ry_right_hip
%   Ry_right_knee
%   Rx_left_hip
%   Ry_left_hip
%   Ry_left_knee ]
%
% outputs 'foot_right' and 'foot_left' are n by 3 vectors respresenting n 
% task space positions with coordinates x,y,z of the feet with respect to 
% the torso
%
% [foot_right,foot_left,knee_right,knee_left,hip_right_x,hip_left_x,...
%                           hip_right_y,hip_left_y] = forward_kinematics(q)
% 
% additional outputs 'knee_right', 'knee_left', hip_right_x', 'hip_left_x',
% 'hip_right_y', 'hip_left_y' are n by 3 vectors respresenting n 
% task space positions with coordinates x,y,z of all joint locations of the
% robot

%% Initialize parameters
param
n = size(q,1);

%% Compute forward kinematics for the robot
% compute positions of right and left torso
torso_right = repmat([0 -torso_width 0],n,1);
torso_left = repmat([0 torso_width 0],n,1);

% compute positions of right and left pelvis
pelvis_right = torso_right+[zeros(n,1) -cos(q(:,1)).*pelvis_width -sin(q(:,1)).*pelvis_width];
pelvis_left = torso_left+[zeros(n,1) cos(q(:,4)).*pelvis_width sin(q(:,4)).*pelvis_width];

% compute positions of right and left hip
hip_right = pelvis_right+[pelvis_depth*ones(n,1) zeros(n,2)];
hip_left = pelvis_left+[pelvis_depth*ones(n,1) zeros(n,2)];

% compute positions of right and left knees
knee_right = hip_right+[-sin(q(:,2))*upper_length sin(q(:,1)).*cos(q(:,2))*upper_length -cos(q(:,1)).*cos(q(:,2))*upper_length];
knee_left = hip_left+[-sin(q(:,5))*upper_length sin(q(:,4)).*cos(q(:,5))*upper_length -cos(q(:,4)).*cos(q(:,5))*upper_length];

% compute positions of right and left feet
ankle_right = knee_right+[-sin(q(:,2)+q(:,3))*lower_length sin(q(:,1)).*cos(q(:,2)+q(:,3))*lower_length -cos(q(:,1)).*cos(q(:,2)+q(:,3))*lower_length];
ankle_left = knee_left+[-sin(q(:,5)+q(:,6))*lower_length sin(q(:,4)).*cos(q(:,5)+q(:,6))*lower_length -cos(q(:,4)).*cos(q(:,5)+q(:,6))*lower_length];