%% Test on Kinematic models
% 'q' is an n by 6 vector representing n joint coordinates:
% [ RHAA RHFE RKFE LHAA LHFE LKFE]
parameters
% load('RightAnkle.mat')
% load('LeftAnkle.mat')
% foot_right=RAnklePlot;
% foot_left=LAnklePlot;

foot_right=[pelvis_depth -((torso_width/2)+pelvis_width) -(upper_length+lower_length+sole_thickness)];
foot_left=[pelvis_depth ((torso_width/2)+pelvis_width) -(upper_length+lower_length+sole_thickness)];
q = inverse_kinematics(foot_right,foot_left)

% %% Tweente Data
% load('s3.mat');
% % Different Velocities: 
% % .cWalking08mps
% % .cWalking04mps
% % .cWalking12mps
% JointAngles = s3.cWalking08mps.average.JointAngles.DataValues; 
% TimeData = s3.cWalking08mps.average.JointAngles.Time; %make sure we get the correct time stamps
% 
% N=length(TimeData);
% for j=1:N
%     t(j,1)=TimeData(1,j);
%     q_tweente(j,1)=JointAngles(3,1,j); % 1 RHIP           
%     q_tweente(j,2)=JointAngles(3,2,j); % 2 LHIP
%     q_tweente(j,4)=JointAngles(3,4,j); % 4 RKNE
%     q_tweente(j,5)=JointAngles(3,5,j); % 5 LKNE
% end 

%% Simulation
Q=real(q);
TORSO=zeros(size(q,1),3);
t = linspace(0,1,size(q,1))';
