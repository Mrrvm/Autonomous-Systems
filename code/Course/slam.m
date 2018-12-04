% SLAM algorithm

% I. INITIALIZATION
%
%   0. System def.
%       q = movement noise
%       m = measurement noise
q = [0.01; 0.1; 0.01; 0.01];
Q = diag(q.^2);
m = [0.25, 2*pi/180];
M = diag(m.^2);
wheelDistance = 0.21;

%   1. Simulator
%       R: robot pose
%       u: control and time=u(5)
%       W: world
R = [0; -2; 0];
u = [0.1; 10*pi/180; 0.1; 0];
W = cloister(-4, 4, -4, 4); %TODO --> Build something cuter
y = zeros(2,size(W,2));

%   2. Estimatior
%       r: robot pose
%       x: map vector
r = [1 2 3];
x = zeros(numel(r)+numel(W), 1); %numel = Rows x Collums
P = zeros(numel(x),numel(x));
x(r) = R;
P(r,r) = 0;
mapspace = 1:numel(x);  %will help us look for new space
mapspace(r) = 0;    %first 3 positions are occupied
l = zeros(2, size(W,2));

%   3. Graphics
mapFig = figure(1);
cla;    %clear axis
axis([-6 6 -6 6])
axis square
WG = line('parent', gca, ...    %landmarks truth
    'linestyle', 'none', ...
    'marker', 'o', ...
    'color', 'r', ...
    'xdata', W(1,:), ...
    'ydata', W(2,:));
RG = line('parent',gca, ...     %simulator robot
    'marker', '*', ...
    'color', 'r', ...
    'xdata', R(1), ...
    'ydata', R(2));
rG = line('parent',gca, ...     %estimator robot
    'marker', '*', ...
    'color', 'b', ...
    'xdata', x(r(1)), ...
    'ydata', x(r(2)));
lG= line('parent',gca, ...     %landmark estimator
    'linestyle', 'none', ...
    'marker', '+', ...
    'color', 'b', ...
    'xdata', [], ...
    'ydata', []);

eG = zeros(1,size(W,2));        %this is for the landmarks elipses
for i = 1:numel(eG)
    eG(i) = line(...
        'parent', gca, ...
        'color', 'g', ...
        'xdata', [], ...
        'ydata', []);
end

reG = zeros(1,size(W,2));        %this is for the robot elipses
for i = 1:numel(eG)
    reG(i) = line(...
        'parent', gca, ...
        'color', 'm', ...
        'xdata', [], ...
        'ydata', []);
end

% II. Temporal Loop

nTimeStamps = 100; %TODO --> change this to fit data
for t = 1:nTimeStamps
    
    %   1. Simulator
    n = q.*randn(4,1);  %robot noise
    R = movement_model(R, [u; t-1], zeros(4,1), t, wheelDistance);
    for lid = 1:size(W,2)
        v = m.*randn(2,1);  %measurement noise
        y(:,lid) = project_point(R, W(:,lid));
    end
    
    %   2. Filter
    %       a. Prediction
    [x(r), R_r, R_n] = movement_model(x(r), [u; t-1], n, t, wheelDistance);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);    %this is sub-optimal, we're using the full matrix
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';  %robot covariance
    
    %       b. Correction
    %           i. known landmarks
    lids = find(l(1,:));
    for lid = lids
        % expectation
        [e, E_r, E_l] = project_point(x(r), x(l(:,lid)));
        E_rl = [E_r E_l];   %Jacobian in respect to robot and landmark
        rl = [r l(:,lid)'];
        E = E_rl * P(rl,rl) * E_rl';
        
        % measurement
        yi = y(:,lid);
        
        % innovation
        z = yi - e;
        if z(2) > pi
            z(2) = z(2) - 2*pi;
        end
        if z(2) < -pi
            z(2) = z(2) + 2*pi;
        end
        Z = M + E;
        
        % Kalman gain
        K = P(:,rl) * E_rl' * Z^-1;
        
        % update
        x = x + K * z;
        P = P - K * Z * K';
    end
    
    %           ii. initialize new landmarks
    %TODO change this below
    %(in this case we are descovering 1 landmark per time stamp)
    l_id = find(l(1,:)==0, 1);   %check landmark id
    if ~isempty(l_id)
        s = find(mapspace, 2);
        if ~isempty(s)
            mapspace(s) = 0;
            l(:,l_id) = s';
            yi = y(:,l_id);
            [x(l(:,l_id)), L_r, L_y] = backProject(x(r), yi);
            P(s,:) = L_r * P(r,:);
            P(:,s) = P(s,:)';
            P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
        end
    end
    
    %   3. Graphics
    set(RG, 'xdata', R(1), 'ydata', R(2)); %simulator robot
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2))); %estimator robot
    lids = find(l(1,:));
    lx = x(l(1,lids));  %landmarks x coordinate
    ly = x(l(2,lids));  %landmarks y coordinate
    set(lG, 'xdata', lx, 'ydata', ly);
    for lid = lids
        el = x(l(:,lid));
        EL = P(l(:,lid),l(:,lid));
        [X,Y] = cov2elli(el,EL,3,16);
        set(eG(lid), 'xdata', X, 'ydata', Y);
    end
    if t > 1
        re = x(r(1:2));
        RE = P(r(1:2),r(1:2));
        [X,Y] = cov2elli(re,RE,3,16);
        set(reG, 'xdata', X, 'ydata', Y);
    end
    
    drawnow;
end

%1:00:30