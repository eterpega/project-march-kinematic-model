function [torso,foot_right,foot_left] = trajectory(n, base, torso_i, torso_f, foot_right_i, foot_right_f, foot_left_i, foot_left_f)

param

foot_height = 0.05; %maximum height of swing foot above the ground
tol = 1e-3; %prevent singularities
l = upper_length+lower_length-tol;

% torso x and y trajectory is a straight line from initial to final position
torso = [linspace(torso_i(1),torso_f(1),n);linspace(torso_i(2),torso_f(2),n)]';

if base == 1 %left leg is swing leg
    % torso height is adapted to keep the stance leg fully stretched
    torso(:,3) = sqrt(l^2+pelvis_width^2-(torso(:,1)+pelvis_depth-torso_i(1)-foot_right_i(1)).^2-(torso(:,2)-torso_width-torso_i(2)-foot_right_i(2)).^2);
    
    % right foot is stance foot
    foot_right = repmat(torso_i+foot_right_i,n,1)-torso;
    
    % left foot x and y trajectory is a straight line from initial to final position
    foot_left = [linspace(foot_left_i(1),foot_left_f(1),n);linspace(foot_left_i(2),foot_left_f(2),n)]';
    
    % left foot z trajectory describes an arc from initial to final foot
    % height. Maximum foot height is determined by parameter foot_height
    midstep = (foot_left_i+torso_i+foot_left_f+torso_f)/2;
    arclength = (midstep-(foot_left_i+torso_i))*(midstep-(foot_left_i+torso_i))';
    radius = (-arclength-foot_height^2)/(-2*foot_height);
    center = [midstep(1:2) -radius+foot_height];
    foot_left(:,3) = sqrt(radius^2-(foot_left(:,1)+torso(:,1)-center(1)).^2-(foot_left(:,2)+torso(:,2)-center(2)).^2)+center(3)-torso(:,3);
else %right leg is swing leg
    % torso height is adapted to keep the stance leg fully stretched
    torso(:,3) = sqrt(l^2+pelvis_width^2-(torso(:,1)+pelvis_depth-torso_i(1)-foot_left_i(1)).^2-(torso(:,2)+torso_width-torso_i(2)-foot_left_i(2)).^2);
    
    % left foot is stance foot
    foot_left = repmat(torso_i+foot_left_i,n,1)-torso;
    
    % right foot x and y trajectory is a straight line from initial to final position
    foot_right = [linspace(foot_right_i(1),foot_right_f(1),n);linspace(foot_right_i(2),foot_right_f(2),n)]';
    
    % right foot z trajectory describes an arc from initial to final foot
    % height. Maximum foot height is determined by parameter foot_height
    midstep = (foot_right_i+torso_i+foot_right_f+torso_f)/2;
    arclength = (midstep-(foot_right_i+torso_i))*(midstep-(foot_right_i+torso_i))';
    radius = (-arclength-foot_height^2)/(-2*foot_height);
    center = [midstep(1:2) -radius+foot_height];
    foot_right(:,3) = sqrt(radius^2-(foot_right(:,1)+torso(:,1)-center(1)).^2-(foot_right(:,2)+torso(:,2)-center(2)).^2)+center(3)-torso(:,3);
end