function [p, P_r, P_pr] = fromFrame(r, p_r)
%point from robot frame to global frame

translation_vector = r(1:2);
alfa = r(3);

Rotation = [cos(alfa) -sin(alfa)
    sin(alfa) cos(alfa)];

p = Rotation*p_r + translation_vector; %first we rotate and then translate

if nargout > 1
        
    px = p_r(1);
    py = p_r(2);
    
    P_r = [ 1, 0, - py*cos(alfa) - px*sin(alfa)
        0, 1,   px*cos(alfa) - py*sin(alfa)];

    P_pr = Rotation;
end

end


%%
function f()
%%
syms x y a px py real
r = [x y a]';
p_r = [px py]';
p = fromFrame(r, p_r);
p_r = jacobian(p,r)
end