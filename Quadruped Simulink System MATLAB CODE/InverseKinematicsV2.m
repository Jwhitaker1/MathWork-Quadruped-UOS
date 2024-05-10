
%robot config for IK
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',7);

L0z = 0.03;
L1z = 0.1;
L2z = 0.450; L2y = 0.180;
L3z = 0.450; L3y = -0.180;
L4z = 0.200; L4y = 0.180;
L5z = 0.125;
L6x = 0.165;

body = rigidBody('link0');
joint = rigidBodyJoint('joint0', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 L0z]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link1');
joint = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint, trvec2tform([0,0,L1z]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link0');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,L2y,L2z]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,L3y,L3z]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link2');


body = rigidBody('link4');
joint = rigidBodyJoint('joint4','revolute');
setFixedTransform(joint, trvec2tform([0,L4y,L4z]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link3');

body = rigidBody('link5');
joint = rigidBodyJoint('joint5','revolute');
setFixedTransform(joint, trvec2tform([0,0,L5z]));
joint.JointAxis = [-1 0 0];
body.Joint = joint;
addBody(robot, body, 'link4');

body = rigidBody('link6');
joint = rigidBodyJoint('joint6','revolute');
setFixedTransform(joint, trvec2tform([-L6x,0,0]));
joint.JointAxis = [-1 0 0];
body.Joint = joint;
addBody(robot, body, 'link5');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link6');


% IK solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1]; % weights for solver

% Target pos and orientation
targetPosition = [-0.18, -1.1559, -0.23]; % desired end-effector pos
targetOrientation = axang2tform([0 1 0 pi]); %desired end-effector orientation
desiredPose = trvec2tform(targetPosition) * targetOrientation;


initialGuessVec = zeros(robot.NumBodies - 1, 1);
[jointConfig, solutionInfo] = ik('tool', desiredPose, weights, initialGuessVec);

% Display robot config
hFig = figure;
show(robot, jointConfig);
title('Initial Robot Configuration');

% Setup time vector and preallocate matrix for joint trajectories
initialTime = 0;
finalTime = 5; % Total time for trajectory
numPoints = 100; % Num of points in trajectory
timeVector = linspace(initialTime, finalTime, numPoints)';
allTrajectories = zeros(numPoints, length(initialGuessVec));

% Calc trajectory for each joint
for i = 1:length(initialGuessVec)
    % Initial + final pos
    p0 = initialGuessVec(i);
    pf = jointConfig(i);
    
    % Zero vel and accel at start and end
    v0 = 0; vf = 0; a0 = 0; af = 0;
    
    % Calc quintic polynomial coeffs
    coeffs = quinticCoeffs(initialTime, finalTime, p0, pf, v0, vf, a0, af);
    
    % Evaluate polynomial at points in time vector
    for j = 1:numPoints
        allTrajectories(j, i) = polyval(flip(coeffs), timeVector(j));
    end
end

% Create a structured array with time and signals for Simulink in degrees
simulinkDataStruct = struct();
for i = 1:size(allTrajectories, 2)
    % Convert the angles from radians to degrees
    anglesInDegrees = rad2deg(allTrajectories(:, i));
    
    % Combine time with joint trajectory in degrees
    jointData = [timeVector, anglesInDegrees]; 
    
    % Assign to struct with field names corresponding to each joint
    simulinkDataStruct.(sprintf('Joint%d', i)) = jointData;
end

% Save the structured array in degrees to the MATLAB base workspace for Simulink
assignin('base', 'simulinkDataStruct', simulinkDataStruct);

% Animate the robot movement once
if ishandle(hFig) % Check if the figure is still open
    for j = 1:numPoints
        show(robot, allTrajectories(j, :)', 'PreservePlot', false);
        title(sprintf('Time: %.2f seconds', timeVector(j)));
        drawnow;
        pause(0.05); % Adjust for smootheness
    end
end

% Function to calc quintic polynomial coeffs
function coeffs = quinticCoeffs(t0, tf, y0, yf, dy0, dyf, a0, a1)
   
    M = [1 t0 t0^2 t0^3 t0^4 t0^5;
         0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
         0 0 2 6*t0 12*t0^2 20*t0^3;
         1 tf tf^2 tf^3 tf^4 tf^5;
         0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
         0 0 2 6*tf 12*tf^2 20*tf^3];
         
    B = [y0; dy0; a0; yf; dyf; a1];
    
    % Solve for coeffs
    coeffs = M \ B;
end



