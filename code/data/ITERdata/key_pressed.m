function key_pressed(src,event)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    
    global loopFlag;
    global speed;
    global Angle1;
    global Angle2;
    disp(event.Key);
    
    if event.Key == 'm'
        CASK_SendRequest('Speed1',0,'Speed2',0);
        loopFlag = false;
        
    elseif strcmp(event.Key,'space')
        speed = 0;
        Angle1 = 0;
        Angle2 = 0;
        CASK_SendRequest('Angle1',Angle1,'Speed1',speed,'Speed2',speed,'Angle2',Angle2);
        
    elseif strcmp(event.Key,'n')
        Angle1 = 0;
        Angle2 = 0;
        CASK_SendRequest('Angle1',Angle1,'Angle2',Angle2);
  
    elseif event.Key == 'w'
        if speed == 0
            speed = 50;
        elseif speed == -50
            speed = 0;
        elseif speed < 130
            speed = speed + 10;
        end
        CASK_SendRequest('Speed1',speed,'Speed2',speed);

    elseif event.Key == 's'        
        if speed == 0
            speed = -50;
        elseif speed == 50
            speed = 0;
        elseif speed > -130
            speed = speed - 10;
        end
        CASK_SendRequest('Speed1',speed,'Speed2',speed);

    elseif event.Key == 'z'
        if Angle1 == 0 && Angle2 == 0
            CASK_SendRequest('Speed1',0,'Speed2',0);
            Angle1 = 270;
            Angle2 = 270;
        elseif Angle1 == 90 && Angle2 == 90
            CASK_SendRequest('Speed1',0,'Speed2',0);
            Angle1 = 0;
            Angle2 = 0;
        end
        CASK_SendRequest('Angle1',Angle1,'Speed1',speed,'Speed2',speed,'Angle2',Angle2);
    
    elseif event.Key == 'c'
        if Angle1 == 0 && Angle2 == 0
            CASK_SendRequest('Speed1',0,'Speed2',0);
            Angle1 = 90;
            Angle2 = 90;
        elseif Angle1 == 270 && Angle2 == 270
            CASK_SendRequest('Speed1',0,'Speed2',0);
            Angle1 = 0;
            Angle2 = 0;
        end
        CASK_SendRequest('Angle1',Angle1,'Speed1',speed,'Speed2',speed,'Angle2',Angle2);
           
    elseif event.Key == 'a'
        if (Angle1-10)>260 || (Angle1-10)<90
            Angle1 = wrapTo360(Angle1-10);
        end
        CASK_SendRequest('Angle1', Angle1);
        
    elseif event.Key == 'd'
        if (Angle1+10)<90 || (Angle1+10)>260
            Angle1 = wrapTo360(Angle1+10);
        end
        CASK_SendRequest('Angle1', Angle1);
        
    elseif event.Key == 'q'
        if (Angle2-10)>260 || (Angle2-10)<90
            Angle2 = wrapTo360(Angle2-10);
        end
        CASK_SendRequest('Angle2', Angle2);
        
    elseif event.Key == 'e'
        if (Angle2+10)<90 || (Angle2+10)>260
            Angle2 = wrapTo360(Angle2+10);
        end
        CASK_SendRequest('Angle2', Angle2);

        
    elseif event.Key == 'o'
        Angle1 = 90;
        CASK_SendRequest('Angle1', Angle1);
        
    elseif event.Key == 'i'
        Angle2 = 270;
        CASK_SendRequest('Angle2', Angle2);
        
    elseif event.Key == 'p'
        speed = 130;
        CASK_SendRequest('Speed1',speed,'Speed2',speed);

    end
end

