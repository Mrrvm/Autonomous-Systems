function [y, Y_r, Y_p] = project_point(r, p)

if nargout == 1
    %without jacobians
    
    p_r = toFrame(r, p);
    y = scan(p_r);
    
else
    %with jacobians
    
    [p_r, PR_r, PR_p] = toFrame(r, p);
    [y, Y_pr] = scan(p_r);
    
    %chain rule
    Y_r = Y_pr * PR_r;
    Y_p = Y_pr * PR_p;


end