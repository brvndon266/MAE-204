%% Mobile Manipulation Project Code Report
% Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
%
% This published document contains the main MATLAB code used for the
% mobile manipulation capstone project, including the trajectory
% generation, feedback controller, state propagation, and full wrapper
% script.

%% TrajectoryGenerator Function
% This function generates the reference end-effector trajectory for the
% pick-and-place task.

type('TrajectoryGenerator.m')

%% FeedbackControl Function
% This function computes the commanded end-effector twist using a
% feedforward-plus-PI control law.

type('FeedbackControl.m')

%% NextState Function
% This function updates the robot configuration using first-order Euler
% integration and wheel odometry.

type('NextState.m')

%% Full Program / Wrapper Script
% This script combines the trajectory generator, controller, and state
% update functions to simulate the complete pick-and-place task.

type('best_main.m')