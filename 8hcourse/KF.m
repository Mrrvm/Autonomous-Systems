% KF Kalman Filter
%
% I. System
%
%   x+ = F_x * x + F_u * u + F_n * n
%   y  = H * x + v
%
%   x : state vector          - P : cov. matrix
%   u : control vector
%   n : perturbation vector   - Q : cov. matrix
%   y : measurement vector
%   v : measurement noise     - R : cov. matrix
%   
%   F_x : transition matrix
%   F_u : control matrix
%   F_n : pert. matrix
%   H   : measurement matrix
%
%
% II. Initialization
%
%   Define F_x, F_u, F_n, and H.
%
%   Precise x, P, Q and R.
%
% III. Temporal loop
%
%   IIIa. Prediction of mean(x) and P at the arrival of u
%       
%       x+ = F_x * x + F_u * u          ... ( + F_n * 0) 
%       P+ = F_x * P * F_x' + F_n * Q * F_n'
%
%   IIIb. Correction of mean(x) and P at the arrival of y
%
%       e = H * x          - expectation
%       E = H * P * H'
%
%       z = y - e          - innovation
%       Z = R + E
%
%       K = P * H' * Z^-1  - Kalman gain
%
%       x+ = x + K * z
%       P+ = P - K * H * P // P - K * Z * K' // and Joseph form
%
% IV. Plot results
%
% V. How to set up KF examples
%
%   1. Simulate system, get x, u and y tranjectories
%   2. Estimate x with the KF. Get x and P trajectories
%   3. Plot results










