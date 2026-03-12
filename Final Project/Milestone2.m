%% Test Script for NextState (Milestone 2)
% Mobile Manipulation Capstone – Scene 6 youBot Kinematics Simulation
%
% This script tests the NextState function by simulating the motion of the
% youBot using constant joint and wheel velocities for one second. The
% resulting robot configurations are written to a CSV file for visualization
% in CoppeliaSim Scene 6.
%
% Written by Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
% brandon.lopez.miramontes@gmail.com

close all;
clear;
clc;

% Initial robot configuration
% [phi; x; y; arm1; arm2; arm3; arm4; arm5; wheel1; wheel2; wheel3; wheel4]
currentState = zeros(12,1);

% Constant controls applied during the simulation
% [5 arm joint speeds; 4 wheel speeds]
jointVel = [0; 0; 0; 0; 0; 10; 10; 10; 10];

% Simulation parameters
dt = 0.01;            % timestep (seconds)
maxJointVel = 15;     % maximum allowable joint/wheel velocity
N = 100;              % number of simulation steps (1 second)

% Preallocate trajectory matrix
traj = zeros(N,13);

% Run the simulation loop
for i = 1:N

    % Compute next robot state
    currentState = NextState(currentState, jointVel, dt, maxJointVel);

    % Store the state (12 variables) + gripper state (0 = open)
    traj(i,:) = [currentState' 0];

end

% Check size of output trajectory
disp('Size of trajectory output:');
disp(size(traj));

% Write CSV file for Scene 6 visualization
writematrix(traj, 'Milestone2_scene6_test.csv');

% Display final robot configuration
disp('Final robot state:');
disp(currentState);

disp('CSV file "Milestone2_scene6_test.csv" created successfully.');