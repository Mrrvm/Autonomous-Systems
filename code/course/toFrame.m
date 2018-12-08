function [p_r, PR_r, PR_p] = toFrame(r, p)
%point from global frame to robot frame

translation_vector = r(1:2);
alfa = r(3);

Rotation = [cos(alfa) -sin(alfa)
    sin(alfa) cos(alfa)];

p_r = Rotation'*(p-translation_vector);

if nargout > 1
    
    px = p(1);
    py = p(2);
    x = translation_vector(1);
    y = translation_vector(2);
    
    PR_r = [ -cos(alfa), -sin(alfa),   cos(alfa)*(py - y) - sin(alfa)*(px - x)
    sin(alfa), -cos(alfa), - cos(alfa)*(px - x) - sin(alfa)*(py - y)];
    
    PR_p = Rotation';
    
end

end

%%
function f()
%% this is to find the jacobian
syms x y a px py real
r = [x y a]';
p = [px py]';
p_r = toFrame(r, p);
PR_r = jacobian(p_r,r)
end