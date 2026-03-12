function nextState = NextState(currentState, jointVel, dt, maxJointVel)
% NextState

% Written by Brandon Lopez
% UCSD Mechatronics & Controls Master's Program
% brandon.lopez.miramontes@gmail.com

% Computes the next configuration of the youBot using first-order Euler
% integration. The state includes chassis configuration, arm joint angles,
% and wheel angles. Wheel motions are also used to update the chassis pose
% through odometry.
%
% Inputs:
%   currentState - 12x1 vector:
%                  [phi; x; y; arm1; arm2; arm3; arm4; arm5; wheel1; wheel2; wheel3; wheel4]
%   jointVel     - 9x1 vector:
%                  [arm1dot; arm2dot; arm3dot; arm4dot; arm5dot; u1; u2; u3; u4]
%   dt           - timestep
%   maxJointVel  - scalar maximum magnitude for all joint and wheel speeds
%
% Output:
%   nextState    - 12x1 vector of updated state


% Ensure column vectors
currentState = currentState(:);
jointVel = jointVel(:);

% Limit all 9 commanded speeds
jointVel = max(-maxJointVel, min(maxJointVel, jointVel));

% Split state
phi = currentState(1);
x = currentState(2);
y = currentState(3);
arm = currentState(4:8);
wheels = currentState(9:12);

% Split controls
armSpeeds = jointVel(1:5);
wheelSpeeds = jointVel(6:9);

% Euler update for arm joints and wheel angles
newArm = arm + armSpeeds * dt;
newWheels = wheels + wheelSpeeds * dt;

% youBot geometry constants
r = 0.0475;      % wheel radius
l = 0.235;       % half-length
w = 0.15;        % half-width

% Wheel angle change
deltaTheta = wheelSpeeds * dt;

% Body twist from wheel motions
F = (r/4) * [ -1/(l+w),  1/(l+w),  1/(l+w), -1/(l+w);
               1,         1,         1,         1;
              -1,         1,        -1,         1 ];

Vb = F * deltaTheta;

wbz = Vb(1);
vbx = Vb(2);
vby = Vb(3);

% Change in chassis configuration in body frame
if abs(wbz) < 1e-9
    deltaQb = [0;
               vbx;
               vby];
else
    deltaQb = [wbz;
               (vbx*sin(wbz) + vby*(cos(wbz)-1)) / wbz;
               (vby*sin(wbz) + vbx*(1-cos(wbz))) / wbz];
end

% Convert body-frame change to space/world frame
T = [1, 0, 0;
     0, cos(phi), -sin(phi);
     0, sin(phi),  cos(phi)];

deltaQ = T * deltaQb;

newChassis = [phi; x; y] + deltaQ;

% Assemble next state
nextState = [newChassis;
             newArm;
             newWheels];
end