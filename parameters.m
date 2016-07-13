floor_length = 4;
floor_width = 2;
floor_height = 0.01;

SEA_stiffness = 80;
ankleFEstiffness = 100;
ankleAAstiffness = 100;

linkheight = 0.075;
linkthickness = 0.025;

torso_width = 111.490833/1000;      % R_torso = 0.25; 
pelvis_width = 135.213424/1000;
pelvis_depth = 207.340833/1000;
upper_length = 467.447184/1000;     % Upper leg: 0.5
lower_length = 567.470264/1000;     % Lower leg: 0.6

%Ankle
distanceFEtoAA = 0.04;
distanceAAtoSole = 0.04;
sole_thickness = 0.04;
sole_width = 0.15;
sole_length = 0.3;

% Physical properties
rho = 1020;

% Foot parameters
foot_width = 0.15;
foot_depth = 0.3;
foot_height = 0.04;
m_foot = 1.21; % [kg]
foot_dims = [foot_width foot_depth foot_height]; % [m]

% Ankle parameters
ankle_offset_height = 0.5*foot_height;
ankle_offset_depth = 0.2*foot_depth;

% Lower leg parameters
R_lower_leg = 0.06; % [m]
L_lower_leg = 0.6; % [m]
m_lower_leg = 4; % [kg]

% Upper leg parameters
R_upper_leg = 0.09; % [m]
L_upper_leg = 0.5; % [m]
m_upper_leg = 9; % [kg]

% Torso parameters
R_torso = 0.25; % [m]
L_torso = 1; % [m]
torso_offset = [0 foot_width L_lower_leg+L_upper_leg+0.5*L_torso];
m_torso = 54.6; % [kg]

% Arm parameters
R_arm = 0.05; % [m]
L_arm = 2.41; % [m]
m_arm = 1; % [kg]

gravity = -9.81; % [m/s^2]
contact_stiffness = 1e7; % [N/m]
contact_damping = 1e4; % [Ns/m]
corner_radius = 0.01; % [m]

t = 0;
q = zeros(1,10);

hanging = 0;

%% Trajectories - Repeating Sequence Stair
Ankle_Trajectory=[0 -6 6 0 0 6 -6 0].';
Knee_Trajectory=[0 30 -30 0 0 -30 30 0].';
Hip_Trajectory=[0 -35 35 0 0 35 -35 0].';