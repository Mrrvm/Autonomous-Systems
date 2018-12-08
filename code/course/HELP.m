% SLAM 2D Simultaneous Localization and Mapping
%
%   Map:
%       x: state vector, with
%           x(r) : robot pose: x(r) = [px, py, a]'  ;  r = [1 2 3]
%           x(l(i,:)): landmark # i: x(l(i,:)) = [xi yi]'
%
%           ex:  x = [x(r) x(l1) x(l2) x(l3) x(l5)]
%
%       P: covariances matrix
%
%       mapspace: [1 2 3 4 5 6 ... N]
%
%       querying map space: s = find(mapspace,size)
%       reserve map space:   mapspace(s) = 0;
%       free map space:      mapspace(s) = s;
%
%   EKF:
%
%       Prediction : when the robot moves
%       Correction : when robot makes a measurement of already mapped lmks
%       Initialization: at the discovery of new lmks
%       (reparametrization): to change lmk nature
%       (deletion): to remove bad lmks
%       
%   Functions: x+ means new x
%       robot motion f()   - x+ = f(x, u ,n)  |   x(r)+ = f(x(r), u, n)
%                                                   F_x = [F_r 0 ; 0 I]
%       lmk oservation h() - y = h(x) + v     |   y = h(x(r), x(l(i,:)))
%                                                   H = [H_r 0 H_i 0]
%       (inverse of h)
%       back-projection g() - x+ = g(x, y)    |   x(l(i,:)) = g(x(r), y)
%                                                   G = [I 0 0; 0 I 0; G_r 0 0]
%
%       Frame transforms:   
%           [l_r, LR_r, LR_l] = toFrame(x(r), x(l(i,:))) = toFrame(R, L) : at lmk correction
%
%           [l, L_r, L_lr] = fromFrame(R, L_R) : at lmk initialization
%
%