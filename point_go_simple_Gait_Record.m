clc
clear
close all

%% Initialize robot parameters
parameters

%% Initialize robot position for first step
n = 100; % number of points in trajectory
torso_height = upper_length+lower_length; % assume constant torso height for the time being
torso_i = [-pelvis_depth torso_width+pelvis_width torso_height]; %initial torso position with respect to world
foot_right_i = [pelvis_depth -torso_width-pelvis_width -torso_height]; %initial right foot position with respect to torso
foot_left_i = [pelvis_depth torso_width+pelvis_width -torso_height]; %initial left foot position with respect to torso

%% Let the robot walk
TORSO = [];
FOOT_RIGHT = [];
FOOT_LEFT = [];
Q = [];
b=[3 1 3 1 3 1 3 1 3 1 3];
b=[1 3 1 3 1 3]; 
for i=1:size(b,2)/2;
  
    % First Step
    dx = [0;450;375;225;150;600;450;375;225;150;600]/1000;
    %dx = [0.2227;0.506;0.4389]
    stones_x = cumsum(dx);
    dy = [0;300;-375;375;-450;450;-300;375;-375;450;-450]/1000;
    %dy = [0.6226;0.4655;0.2266]
    stones_y = cumsum(dy);
    point_x = stones_x(i);
    point_y = stones_y(i);  
    base =b(i);
    point = [point_x point_y 0]-torso_i; %next step location with respect to torso
    
    % compute final task space positions dependent on right or left swing foot
    if base == 1 %left swing leg
        torso_d = (point+foot_right_i)/2-[pelvis_depth 0 0]; %keep torso inbetween stance and swing foot
        torso_d(3) = 0; %do not change torso height for the time being
        foot_right_f = foot_right_i-torso_d; %final right foot position with respect to torso, just coordinate shift with the new location of the torso, no movemente of the foot
        foot_left_f = point-torso_d; %final left foot position with respect to torso, new position of torso
    elseif base == 3 %right swing leg
        torso_d = (point+foot_left_i)/2-[pelvis_depth 0 0]; %keep torso inbetween stance and swing foot
        torso_d(3) = -1; %do not change torso height for the time being
        foot_left_f = foot_left_i-torso_d; %final left foot position with respect to torso
        foot_right_f = point-torso_d; %final right foot position with respect to torso
%    else
%        break
    end
    torso_f = torso_i+torso_d; %final torso position with respect to world
    
    % compute task space trajectories from initial to final positions
    % torso trajectory is with respect to world, feet trajectories are with respect to torso
    [torso,foot_right,foot_left] = trajectory(n,base,torso_i,torso_f,foot_right_i,foot_right_f,foot_left_i,foot_left_f);
    
    % store task space trajectories
    TORSO = [TORSO;torso];
    FOOT_RIGHT = [FOOT_RIGHT;foot_right];
    FOOT_LEFT = [FOOT_LEFT;foot_left];
    
    % compute joint trajectories using inverse kinematics
    q = inverse_kinematics(foot_right,foot_left);
    
    % store joint space trajectories
    Q = [Q;q];
    
    % compute initial task space positions for next step
    torso_i = torso_f;
    foot_right_i = foot_right_f;
    foot_left_i = foot_left_f;
    
end
    %Change of angle reference to match with recordings
    Q = -Q;
    t = linspace(0,5,i*n)';
        
%         %Load Data    
%      load('gait1_2steps.mat')
%      t=DATA_deg(1500:2200,1);
%      TORSO=zeros(size(t,1),3);
%  
%     Q=zeros(size(t,1),6);
%     Q(:,2)=(pi/180)*DATA_deg(1500:2200,2);
%     Q(:,3)=(pi/180)*DATA_deg(1500:2200,3);
%     Q(:,5)=(pi/180)*DATA_deg(1500:2200,4);
%     Q(:,6)=(pi/180)*DATA_deg(1500:2200,5);

    load('gait1_2steps')
    t=DATA_deg(:,1);
    TORSO=zeros(size(t,1),3);
 
    Q=zeros(size(t,1),6);
    Q(:,2)=(pi/180)*DATA_deg(:,2);
    Q(:,3)=(pi/180)*DATA_deg(:,3);
    Q(:,5)=(pi/180)*DATA_deg(:,4);
    Q(:,6)=(pi/180)*DATA_deg(:,5);

 % figure(1);subplot(2,1,1);plot(right_foot_out);subplot(2,1,2);plot(left_foot_out);