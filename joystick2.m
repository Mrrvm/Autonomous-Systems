disp('antes init')
CASK_Init();
disp('depois init')

CASK_SendRequest('Angle1', 0, 'Angle2', 0);


h_fig = figure;
set(h_fig, 'KeyPressFcn', @fun);

global loopFlag;
loopFlag = true;

global speed;
speed=0;
global Angle1;
global Angle2;
Angle1=0;
Angle2=0;


disp('antes ciclo')
while loopFlag
    if loopFlag == false
        break;
    end
    pause(0.5);
end
disp('saiu ciclo')

close all;
CASK_CloseComm();