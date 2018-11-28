% send keyboard keys and read fwAngle, fwVelocity, bwAngle, bwVelocity
% [fwAngle, fwVelocity, bwAngle, bwVelocity, landmarkID, landmarkDist, landmarkAngle] * nTimestamps
% fw is front wheels, bw is back wheels

format long

pauseCNT = 0.1;
wheelDiameter = 0.0304;
maxSpeed = 130;
speedThresh = 10;

CASK_Init();

CASK_SendRequest('Angle1', 0, 'Angle2', 0);

h_fig = figure;
set(h_fig, 'KeyPressFcn', @key_pressed);

global loopFlag speed Angle1 Angle2;
loopFlag = true;
speed = 0;
Angle1 = 0;
Angle2 = 0;

c = clock;
data = struct('timestamp', (c(4)*60 + c(5))*60 + c(6),'odom', zeros(1,4));

while loopFlag
    if loopFlag == false
        break;
    end
    
    
    [t,bat,fwAngle,bwAngle,fwTacho,fwSpeed,bwTacho,bwSpeed,status]=CASK_ReadValues;
   
    if status == 0 && abs(fwSpeed) < (maxSpeed + speedThresh) && abs(bwSpeed) < (maxSpeed + speedThresh)
        
        fwSpeed = (double(fwSpeed)*2*pi*wheelDiameter)/60;
        bwSpeed = (double(bwSpeed)*2*pi*wheelDiameter)/60;
        c = clock;
        data = [data; struct('timestamp', (c(4)*60 + c(5))*60 + c(6),'odom', [fwSpeed double(fwAngle) bwSpeed double(bwAngle)])];
                    
    end
    
    pause(pauseCNT);
end


save('iterdata.mat', 'data');

close all;
CASK_CloseComm();
