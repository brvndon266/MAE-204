% Test Script for FeedbackControl (Component 3)
% Mobile Manipulation Capstone – Feedforward Plus Feedback Control
%
% This script tests the FeedbackControl function using the recommended
% example values from the capstone instructions. It computes the commanded
% end-effector twist, the configuration error twist, and the updated
% accumulated error term.
%
% Written by Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
% brandon.lopez.miramontes@gmail.com

close all;
clear;
clc;

% Current actual end-effector configuration X
T_se = [0.170  0      0.985  0.387;
        0      1      0      0;
       -0.985  0      0.170  0.570;
        0      0      0      1];

% Current reference end-effector configuration X_d
T_se_d = [0  0  1  0.5;
          0  1  0  0;
         -1  0  0  0.5;
          0  0  0  1];

% Reference end-effector configuration at next timestep X_d,next
T_se_d_next = [0  0  1  0.6;
               0  1  0  0;
              -1  0  0  0.3;
               0  0  0  1];

% Control gains
Kp = zeros(6,6);
Ki = zeros(6,6);

% Timestep
dt = 0.01;

% Initial accumulated error
Verr_accu = zeros(6,1);

% Call FeedbackControl
[Vb, Verr_accu_new, Verr] = FeedbackControl(T_se, T_se_d, T_se_d_next, ...
                                            Kp, Ki, dt, Verr_accu);

% Compute Vd separately for display
Vd = se3ToVec(MatrixLog6(TransInv(T_se_d) * T_se_d_next)) / dt;

% Compute adjoint-mapped feedforward term for display
Ad_term = Adjoint(TransInv(T_se) * T_se_d) * Vd;

% Display results
disp('Computed desired twist Vd:');
disp(Vd);

disp('Computed adjoint mapped twist Ad_X^-1Xd * Vd:');
disp(Ad_term);

disp('Computed configuration error Xerr:');
disp(Verr);

disp('Computed commanded twist V:');
disp(Vb);

disp('Updated accumulated error integral:');
disp(Verr_accu_new);

% Compute the mobile manipulator Jacobian Je

% Arm joint angles from the test configuration
theta = [0; 0; 0.2; -1.6; 0];

% youBot geometry parameters
l = 0.47 / 2;
w = 0.3 / 2;
r = 0.0475;

% Wheel Jacobian matrix F
F = (r/4) * [ -1/(l+w),   1/(l+w),   1/(l+w),  -1/(l+w);
               1,          1,          1,         1;
              -1,          1,         -1,         1 ];

% Expand F into a 6x4 matrix
F6 = [0 0 0 0;
      0 0 0 0;
      F(1,:);
      F(2,:);
      F(3,:);
      0 0 0 0];

% Fixed transform from chassis frame {b} to arm base frame {0}
T_b0 = [1 0 0 0.1662;
        0 1 0 0;
        0 0 1 0.0026;
        0 0 0 1];

% Home configuration of the end-effector relative to frame {0}
M_0e = [1 0 0 0.033;
        0 1 0 0;
        0 0 1 0.6546;
        0 0 0 1];

% Body screw axes of the 5-joint arm
Blist = [0  0  1  0       0.033   0;
         0 -1  0 -0.5076  0       0;
         0 -1  0 -0.3526  0       0;
         0 -1  0 -0.2176  0       0;
         0  0  1  0       0       0]';

% Arm Jacobian
Jarm = JacobianBody(Blist, theta);

% Current end-effector pose relative to frame {0}
T_0e = FKinBody(M_0e, Blist, theta);

% Base Jacobian
T_eb = TransInv(T_0e) * TransInv(T_b0);
Jbase = Adjoint(T_eb) * F6;

% Full mobile manipulator Jacobian
Je = [Jbase Jarm];

disp('Computed mobile manipulator Jacobian Je:');
disp(Je);

% Compute control vector [u; thetadot]

controls = pinv(Je) * Vb;

u = controls(1:4);
thetadot = controls(5:9);

disp('Computed wheel speeds u:');
disp(u);

disp('Computed arm joint speeds thetadot:');
disp(thetadot);

disp('Combined control vector [u; thetadot]:');
disp(controls);