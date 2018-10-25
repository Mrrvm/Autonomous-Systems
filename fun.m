function fun(src,event)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    global loopFlag;
    disp(event.Key);
    
    if event.Key == 'q'
        loopFlag = false;
    end
end

