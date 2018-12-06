load('/home/imarcher/Dropbox/Tecnico/SA/code/data/ITERdata/bags/iterdata1.mat');
iterdata = data; clear data;
matObj = matfile('/home/imarcher/Dropbox/Tecnico/SA/code/data/ITERdata/bags/iterdata1.mat');
details = whos(matObj);
aux = details.size;
iterdataSize = aux(1);

cameraDataFile = fopen('/home/imarcher/Dropbox/Tecnico/SA/code/data/CameraData/landmark.txt','r');

i = 1;
while true
  thisline = fgetl(cameraDataFile);
  if ~ischar(thisline)
      break; 
  end  
  brokenline = strsplit(thisline);
  [m, r] = strtok(brokenline(1), '.');
  hour = str2double(m);
  [m, r] = strtok(r, '.');
  min = str2double(m);
  [m, r] = strtok(r, '.');
  sec = str2double(strcat(m,r));
  cam(i).timestamp = hour*3600+min*60+sec;
  cam(i).landmarksSeen =  str2double(brokenline(2));

  c = 3;
  for j = 1:cam(i).landmarksSeen
      cam(i).landmark(j, :) = [str2double(brokenline(c)) str2double(brokenline(c+1)) str2double(brokenline(c+2))];
      c = c+3;  
  end
  
  i = i+1;
end
fclose(cameraDataFile);

cameradataSize = i-1;

i = 1;
k = 1;
j = 1;
while j <= cameradataSize 
    if i <= iterdataSize
        tIter = iterdata(i).timestamp;  
        tCam = cam(j).timestamp;
        if tCam <= tIter
            data(k).time = tCam;
            data(k).option = 1;
            data(k).landmarksSeen = cam(j).landmarksSeen;
            data(k).landmark = cam(j).landmark;
            k = k + 1;
            j = j + 1;
        end
        if tCam > tIter 
            data(k).time = tIter;
            data(k).option = 0;
            data(k).odom(:) = iterdata(i).odom;
            k = k + 1;
            i = i + 1;
        end
    else
        data(k).time = tCam;
        data(k).option = 1;
        data(k).landmarksSeen = cam(j).landmarksSeen;
        data(k).landmark = cam(j).landmark;
        k = k + 1;
        j = j + 1;
    end
end

while i <= iterdataSize
    data(k).time = iterdata(i).timestamp;
    data(k).option = 0;
    data(k).odom(:) = iterdata(i).odom;
    k = k + 1;
    i = i + 1;
end

k = 1;
while k <= size(data,2)
    if data(k).option == 0
        data(k).odom(2) = pi*data(k).odom(2)/180;
        data(k).odom(4) = pi*data(k).odom(4)/180;
    end
    k = k + 1;
end


