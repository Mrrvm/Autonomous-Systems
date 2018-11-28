clear all
close all
clc
format long

load('/home/imarcher/Dropbox/Tecnico/SA/code/data/ITERdata/iterdata_fulldate.mat');


dates = data.date;
timestamp = (([dates.hour].*60 + [dates.min]).*60 + [dates.seg])';
odom = [data.fwSpeed data.fwAngle data.bwSpeed data.bwAngle];

len = length(dates);

for i = 1:len
    iterdata(i,1) = struct('time', timestamp(i),'odom', odom(i,:));
end

save('iterdata.mat', 'iterdata');
