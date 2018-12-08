function [p, P_y] = invScan(y)
% y = [range, alfa]
% p = [x, y] on robot frame


range = y(1);
alfa = y(2);

px = range*cos(alfa);
py = range*sin(alfa);

p = [px;py];

if nargout > 1
    
    P_y = [cos(alfa) -range*sin(alfa)
        sin(alfa) range*cos(alfa)];
    
end