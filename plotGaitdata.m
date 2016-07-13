clear; close all; 

%% Data Description
% 1 RHIP           
% 2 LHIP           
% 3 HAPE           
% 4 RKNE           
% 5 LKNE           
% 6 RANK           
% 7 LANK  

%% Load Data and Simulink Parametes
% parameters
load('s3.mat');
% Different Velocities: 
% .cWalking08mps
% .cWalking04mps
% .cWalking12mps
JointAngles = s3.cWalking04mps.average.JointAngles.DataValues; 
TimeData = s3.cWalking04mps.average.JointAngles.Time; %make sure we get the correct time stamps

N=length(TimeData);
for j=1:N
    t(j,1)=TimeData(1,j);
    % 1 RHIP           
    q1(j,1)=JointAngles(3,1,j);
    % 2 LHIP           
    q2(j,1)=JointAngles(3,2,j);
    % 4 RKNE           
    q4(j,1)=JointAngles(3,4,j);
    % 5 LKNE           
    q5(j,1)=JointAngles(3,5,j);
    % 6 RANK           
    q6(j,1)=JointAngles(3,6,j);
    % 7 LANK          
    q7(j,1)=JointAngles(3,7,j);
end 

% Rewrite Data in the vector Q as input of simulink
Q = [zeros(N,1) q1 q4 zeros(N,1) q2 q5];
TORSO = zeros(size(t,1),3);

%% Plot Data
figure()
% 1 RHIP           
subplot(322); 
plot(t,q1,'b.');
title('Right HFE')
% 2 LHIP           
subplot(321);
plot(t,q2,'b.');
title('Left HFE')
% 4 RKNE           
subplot(324);
plot(t,q4,'b.');
title('Right KFE')
% 5 LKNE           
subplot(323);
plot(t,q5,'b.');

% 6 RANK           
subplot(326);
plot(t,q4,'b.');
title('Right ANK')
% 7 LANK           
subplot(325);
plot(t,q5,'b.');
title('Left ANK')

xlabel('Time')
ylabel('Angle [rad]')

          
% figure();
% % 6 RANK           
% subplot(121);
% plot3(p6(1,:),p6(3,:),p6(2,:));
% title('Right ANK')
% % 7 LANK           
% subplot(122);
% plot3(p7(1,:),p7(3,:),p7(2,:));
% title('Left ANK')

hold off
%% Position of EndEffector (Ankle)
% Right End-Effector (Right_Ankle) Positioning through time #
% Used the Walking 08mps - Average Joint Position
% Take all joint position values
Joint_Position = s3.cWalking08mps.average.JointPosition.DataValues;
% For the right ankle, the matrix value is in column 6 - saving in
% position_right_end_effector variable (p_rende)
pos_RANK = Joint_Position(:,6,:);
pos_LANK = Joint_Position(:,7,:);

for j=1:size(pos_RANK,3)
    % 6 Right ANK Position           
    p_RANK(:,j)=pos_RANK(:,:,j)-[0.2711; 0.1122; 0.0572];
    % 7 Left ANK Position           
    p_LANK(:,j)=pos_LANK(:,:,j);
end 

figure();
subplot(121)
plot3(p_RANK(1,:),p_RANK(3,:),p_RANK(2,:));
title('Right ANK');xlabel('X');ylabel('Y');zlabel('Z');
grid on
subplot(122)
plot3(p_LANK(1,:),p_LANK(3,:),p_LANK(2,:));
title('Left ANK');xlabel('X');ylabel('Y');zlabel('Z');
grid on

LAnkGait=[];
LAnkGait=[p_LANK(1,:)' p_LANK(2,:)' p_LANK(3,:)']

RAnkGait=[];
RAnkGait=[p_RANK(1,:)' p_RANK(2,:)' p_RANK(3,:)']


% %%
% % Ploting the end-effector time and space line
% % Recorded data is repeted to have a bigger time to analyze
% for n=1:4
%     Q=[Q; Q];
%     t2=t+t(end);
%     t=[t;t2]; 
%     TORSO=[TORSO;TORSO];
% end
