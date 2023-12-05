% Walking Robot Parameters
% Copyright 2017-2019 The MathWorks, Inc.

%% Actuator type
actuatorType = 1; % 1:motion controlled, 2:torque controlled, 3:motor controlled

%% Contact and friction parameters
contact_stiffness = 400/0.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)

height_plane = 0.025;
plane_z = height_plane; 
plane_x = 3;
plane_y = 50;
contact_point_radius = 0.0001; %m


%% Robot mechanical Parameters (m)
% body.x_length = 1; %was 0.6
% body.y_length = 0.3*1.5; %was 0.3
% body.z_length = 0.15; %was 0.15
% body.shoulder_size = 0.07; %was 0.07 
% body.upper_length = 0.35; %was 0.16
% body.lower_length = 0.25; %was 0.25
% body.foot_radius = 0.035; %was 0.035
% body.shoulder_distance = 0.4; %was 0.4
% body.shoulder_radius = l1*0.025;
% body.pin_radius = l1*0.025;
% body.pin_offset = 0.0192;
% body.max_stretch = body.upper_length + body.lower_length;
% body.knee_damping = 0.1;

density = 1200;

leg_width = 0.035;
lower_leg_length = 0.27; 
upper_leg_length = 0.27;

foot_radius = 0.045;
foot_z = 0.1;

torso_width = 0.4; 
torso_length = 1; 
torso_height = 0.25; 

torso_offset_height = 0; 
torso_offset_length = 0; 

world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

%% Initial conditions
% Height of the 6-DOF joint between the ground and robot torso
init_height = lower_leg_length + upper_leg_length + torso_offset_height ...
              + foot_z/2 + plane_z/2;  
% Joint angles [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
init_angs_R = zeros(6,1);
init_angs_L = zeros(6,1);
          
%% Robot joint parameters
joint_damping = 1;
motion_time_constant = 0.001;

%% Joint controller parameters
hip_servo_kp = 12500;
hip_servo_ki = 3500;
hip_servo_kd = 100;

knee_servo_kp = 10000;
knee_servo_ki = 2750;
knee_servo_kd = 75;


deriv_filter_coeff = 1000;
max_torque = 80;

%% Electric motor parameters
motor_voltage = 24;
motor_resistance = 8;
motor_constant = 0.16;
motor_inertia = 1e-5;
motor_damping = 1e-5;
motor_inductance = 250e-6;
gear_ratio = 100;
