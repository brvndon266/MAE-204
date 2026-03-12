%% Test Script for TrajectoryGenerator (Milestone 1)
% Mobile Manipulation Capstone – Scene 8 End-Effector Animation
%
% This script tests the TrajectoryGenerator function by generating a
% pick-and-place reference trajectory and exporting the result as a CSV
% file for visualization in CoppeliaSim Scene 8.
%
% Written by Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
% brandon.lopez.miramontes@gmail.com

close all;
clear;
clc;

% Initial end-effector configuration from the capstone page
T_se_init = [0 0 1 0;
             0 1 0 0;
            -1 0 0 0.5;
             0 0 0 1];

% Default cube initial configuration
T_sc_init = [1 0 0 1;
             0 1 0 0;
             0 0 1 0.025;
             0 0 0 1];

% Default cube final configuration
T_sc_final = [0 1 0 0;
             -1 0 0 -1;
              0 0 1 0.025;
              0 0 0 1];

% Grasp configuration of the end-effector relative to the cube
% End-effector z-axis points downward toward the cube
T_ce_grasp = [-sqrt(2)/2  0  sqrt(2)/2   0;
               0          1  0           0;
              -sqrt(2)/2  0 -sqrt(2)/2   0;
               0          0  0           1];

% Standoff configuration a little above the cube
T_ce_standoff = [-sqrt(2)/2  0  sqrt(2)/2   0;
                  0          1  0           0;
                 -sqrt(2)/2  0 -sqrt(2)/2   0.10;
                  0          0  0           1];

% Generate trajectory
trajOutput = TrajectoryGenerator(T_se_init, T_sc_init, T_sc_final, T_ce_grasp, T_ce_standoff);


% Check size
disp('Size of trajectory output:');
disp(size(trajOutput));

% Write CSV for Scene 8
writematrix(trajOutput, 'Milestone1_scene8_test.csv');

disp('CSV file "Milestone1_scene8_test.csv" created successfully.');