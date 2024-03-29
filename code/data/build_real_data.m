clear all;
close all;

PATH = '/home/imarcher/Dropbox/Tecnico/SA/code/data/';


fileList = dir('/home/imarcher/Dropbox/Tecnico/SA/code/data/CameraData/*.txt');
for i=1:size(fileList,1)
    [m, r] = split(fileList(i).name, 'landmark');
    m = split(m(2), '.txt');
    torun(i) = m(1);
end
size = size(fileList,1);

    
for x=1:size
    load(strcat(PATH,'ITERdata/','iterdata',string(torun(x)),'.mat'));
    strcat(PATH,'ITERdata/','iterdata',string(torun(x)),'.mat')
    iterdataSize = length(iterdata);

    cameraDataFile = fopen(strcat(PATH,'CameraData/','landmark',string(torun(x)),'.txt'),'r');
    strcat(PATH,'CameraData/','landmark',string(torun(x)),'.txt')
    
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

    save(strcat(PATH,'data',string(torun(x)),'.mat'), 'data');
    strcat(PATH,'data',string(torun(x)),'.mat')
    clearvars -except torun x PATH
end