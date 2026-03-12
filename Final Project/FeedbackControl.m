function [Vb, Verr_accu, Verr] = FeedbackControl(T_se, T_se_d, T_se_d_next, Kp, Ki, dt, Verr_accu)
% FeedbackControl

% Written by Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
% brandon.lopez.miramontes@gmail.com

% This function computes the commanded end-effector twist using a
% feedforward-plus-feedback control law. The controller combines the
% reference twist needed to move from the current desired configuration
% to the next desired configuration with proportional-integral correction
% based on the current end-effector error.
%
% Inputs:
%   T_se        - Current actual end-effector configuration X (4x4)
%   T_se_d      - Current desired end-effector configuration X_d (4x4)
%   T_se_d_next - Desired end-effector configuration at the next timestep
%                 X_d,next (4x4)
%   Kp          - Proportional gain matrix (6x6)
%   Ki          - Integral gain matrix (6x6)
%   dt          - Timestep between reference trajectory configurations
%   Verr_accu   - Accumulated end-effector error integral (6x1)
%
% Outputs:
%   Vb          - Commanded end-effector twist expressed in frame {e} (6x1)
%   Verr_accu   - Updated accumulated error integral (6x1)
%   Verr        - Current end-effector error twist X_err (6x1)
%


% Step 1: Compute the feedforward reference twist Vd
Vd_M = TransInv(T_se_d) * T_se_d_next;
Vd = se3ToVec(MatrixLog6(Vd_M)) / dt;

% Step 2: Compute the current configuration error twist X_err
Verr_M = TransInv(T_se) * T_se_d;
Verr = se3ToVec(MatrixLog6(Verr_M));

% Step 3: Compute the adjoint transformation
AdT = Adjoint(TransInv(T_se) * T_se_d);

% Step 4: Update the accumulated integral error
Verr_accu = Verr_accu + Verr * dt;

% Step 5: Compute the feedforward-plus-PI control law
Vb = AdT * Vd + Kp * Verr + Ki * Verr_accu;

end