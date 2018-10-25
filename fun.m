function fun(src,event)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    global loopFlag;
    global speed;
    global Angle1;
    global Angle2;
    disp(event.Key);
    
    if event.Key == 'q'
        CASK_SendRequest('Speed1',0,'Speed2',0);
        loopFlag = false;
        
    elseif strcmp(event.Key,'space')
        speed=0
        Angle1=0
        Angle2=0
        CASK_SendRequest('Speed1',speed,'Speed2',speed);
        CASK_SendRequest('Angle1', Angle1, 'Angle2', Angle2);
  
    elseif event.Key == 'w'
        if speed==0
            speed=50
        elseif speed==-50
            speed=0
        elseif speed<130
            speed=speed+10
        end
        CASK_SendRequest('Speed1',speed,'Speed2',speed);

        
    elseif event.Key == 's'        
        if speed==0
            speed=-50
        elseif speed==50
            speed=0
        elseif speed>-130
            speed=speed-10
        end
        CASK_SendRequest('Speed1',speed,'Speed2',speed);

        
    elseif event.Key == 'a'
        if Angle1==0 && Angle2==0;
            Angle1=270
            Angle2=270
        elseif Angle1==90 && Angle2==90;
            Angle1=0
            Angle2=0
        end
        CASK_SendRequest('Angle1', Angle1, 'Angle2', Angle2);
    
    elseif event.Key == 'd'
        if Angle1==0 && Angle2==0;
            Angle1=90
            Angle2=90
        elseif Angle1==270 && Angle2==270;
            Angle1=0
            Angle2=0
        end
        CASK_SendRequest('Angle1', Angle1, 'Angle2', Angle2);
        
        
    %elseif event.Key == 'q'
    %   Angle1=wrapTo360(

    %    CASK_SendRequest('Angle1', Angle1, 'Angle2', Angle2);

    end
end

