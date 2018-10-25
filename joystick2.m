CASK_Init();

CASK_SendRequest('Angle1', 0, 'Angle2', 0);


h_fig = figure
set(h_fig, 'KeyPressFcn', @fun);

global loopFlag;
loopFlag = true;

disp('antes ciclo')
while loopFlag
    if loopFlag == false
        break;
    end
    pause(0.5);
end
disp('saiu ciclo')


CASK_CloseComm();