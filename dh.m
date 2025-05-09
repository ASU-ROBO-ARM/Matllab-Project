% Script to extract Denavit-Hartenberg parameters from a Simscape Multibody model
% This script helps you calculate the DH table for a robotic manipulator

%% Load and prepare the Simscape model
% Replace 'your_model_name' with the name of your Simscape Multibody model
modelName = 'Assembly_Model';

% Try to load the model if it's not already loaded
if ~bdIsLoaded(modelName)
    try
        load_system(modelName);
    catch
        error('Could not load the specified model. Please ensure the model name is correct.');
    end
end

%% Find all joints in the model
allBlocks = find_system(modelName, 'Type', 'Block');
jointBlocks = find_system(modelName, 'RegExp', 'on', 'BlockType', 'SubSystem', 'Name', '.*Joint.*');

% Also look for specific joint types
revoluteJoints = find_system(modelName, 'BlockType', 'SubSystem', 'MaskType', 'Revolute Joint');
prismaticJoints = find_system(modelName, 'BlockType', 'SubSystem', 'MaskType', 'Prismatic Joint');

jointBlocks = unique([jointBlocks; revoluteJoints; prismaticJoints]);

fprintf('Found %d potential joint blocks in the model.\n', length(jointBlocks));

%% Extract the joint frames and parameters
% Initialize DH parameter table
dhParams = struct('jointName', {}, 'jointType', {}, 'a', {}, 'alpha', {}, 'd', {}, 'theta', {});

% Process each joint
for i = 1:length(jointBlocks)
    jointBlock = jointBlocks{i};
    
    % Get joint name
    [~, jointName] = fileparts(jointBlock);
    
    % Determine joint type (revolute or prismatic)
    if contains(lower(get_param(jointBlock, 'MaskType')), 'revolute')
        jointType = 'revolute';
    elseif contains(lower(get_param(jointBlock, 'MaskType')), 'prismatic')
        jointType = 'prismatic';
    else
        fprintf('Skipping %s as it is not a recognized joint type.\n', jointName);
        continue;
    end
    
    % Get joint parameters
    try
        % This is where we would extract the actual parameters
        % In a real implementation, we would get the frame transforms
        % For this example, we'll use placeholder values
        
        % Get the coordinate frames connected to this joint
        % In Simscape, frames are represented by connection ports
        
        % Get positions of the connected frames
        % These need to be determined from your specific model structure
        % Example (these would actually come from your model):
        frame1Position = [0, 0, 0]; % Base frame
        frame2Position = [0, 0, 0]; % Next frame
        
        % Calculate the DH parameters from the frame transformations
        % This is a simplified example - actual calculation depends on your model
        
        % For demonstration purposes:
        a = 0;      % Link length
        alpha = 0;  % Link twist
        d = 0;      % Link offset
        theta = 0;  % Joint angle
        
        % Store the parameters
        dhParams(end+1).jointName = jointName;
        dhParams(end).jointType = jointType;
        dhParams(end).a = a;
        dhParams(end).alpha = alpha;
        dhParams(end).d = d;
        dhParams(end).theta = theta;
        
    catch ex
        fprintf('Error extracting parameters for joint %s: %s\n', jointName, ex.message);
    end
end

%% Function to extract actual DH parameters from frame transforms
% This would be a proper implementation for your specific model structure
function [a, alpha, d, theta] = extractDHFromFrames(frame1, frame2)
    % Calculate the transformation between frames
    % This is a placeholder - real implementation depends on your model
    
    % Link length - distance from Zi-1 to Zi measured along Xi-1
    a = 0;
    
    % Link twist - angle between Zi-1 and Zi measured about Xi-1
    alpha = 0;
    
    % Link offset - distance from Xi-1 to Xi measured along Zi
    d = 0;
    
    % Joint angle - angle between Xi-1 and Xi measured about Zi
    theta = 0;
end

%% Display the DH parameter table
fprintf('\nDenavit-Hartenberg Parameters Table:\n');
fprintf('%-20s %-12s %-8s %-8s %-8s %-8s\n', 'Joint Name', 'Joint Type', 'a', 'alpha', 'd', 'theta');
fprintf('%-20s %-12s %-8s %-8s %-8s %-8s\n', '---------', '---------', '-', '-----', '-', '-----');

for i = 1:length(dhParams)
    fprintf('%-20s %-12s %-8.4f %-8.4f %-8.4f %-8.4f\n', ...
        dhParams(i).jointName, dhParams(i).jointType, ...
        dhParams(i).a, dhParams(i).alpha, dhParams(i).d, dhParams(i).theta);
end

%% Function to compute forward kinematics using the DH parameters
function T = forwardKinematics(dhParams, jointValues)
    % Initialize transformation matrix
    T = eye(4);
    
    % Apply each joint transformation
    for i = 1:length(dhParams)
        % Get DH parameters for this joint
        a = dhParams(i).a;
        alpha = dhParams(i).alpha;
        d = dhParams(i).d;
        theta = dhParams(i).theta;
        
        % If this is a revolute joint, update theta with the joint value
        if strcmp(dhParams(i).jointType, 'revolute')
            theta = jointValues(i);
        % If this is a prismatic joint, update d with the joint value
        elseif strcmp(dhParams(i).jointType, 'prismatic')
            d = jointValues(i);
        end
        
        % Calculate the transformation matrix for this joint
        ct = cos(theta);
        st = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        
        Ti = [
            ct, -st*ca, st*sa, a*ct;
            st, ct*ca, -ct*sa, a*st;
            0, sa, ca, d;
            0, 0, 0, 1
        ];
        
        % Update the total transformation
        T = T * Ti;
    end
end

%% Example of using the extracted DH parameters
% Set joint values (angles for revolute joints, displacements for prismatic)
jointValues = zeros(1, length(dhParams)); % Default positions

% Calculate forward kinematics
T = forwardKinematics(dhParams, jointValues);

fprintf('\nForward Kinematics at default position:\n');
disp(T);

%% Visualize the robot
figure;
hold on;
grid on;
title('Robot Kinematic Chain');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);

% Initialize position
p = [0; 0; 0];
plotPoint(p, 'ro', 'Base');

% Current transformation
curT = eye(4);

% Draw each link
for i = 1:length(dhParams)
    a = dhParams(i).a;
    alpha = dhParams(i).alpha;
    d = dhParams(i).d;
    theta = dhParams(i).theta;
    
    % Update for joint value if needed
    if strcmp(dhParams(i).jointType, 'revolute')
        theta = jointValues(i);
    elseif strcmp(dhParams(i).jointType, 'prismatic')
        d = jointValues(i);
    end
    
    % Calculate the transformation matrix for this joint
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    
    Ti = [
        ct, -st*ca, st*sa, a*ct;
        st, ct*ca, -ct*sa, a*st;
        0, sa, ca, d;
        0, 0, 0, 1
    ];
    
    % Update total transformation
    curT = curT * Ti;
    
    % Plot the new position
    newP = curT(1:3, 4);
    line([p(1), newP(1)], [p(2), newP(2)], [p(3), newP(3)], 'Color', 'b', 'LineWidth', 2);
    plotPoint(newP, 'go', sprintf('Joint %d', i));
    
    % Update position
    p = newP;
end

function plotPoint(p, style, label)
    plot3(p(1), p(2), p(3), style, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(p(1), p(2), p(3), label);
end