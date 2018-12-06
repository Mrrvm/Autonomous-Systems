load('data1.mat');

wheeldistance = 0.21;
rNoise = [0.1; 0.0175; 0.1; 0.0175];
Rn = diag(rNoise.^2);
pose = [0 0 0];
last_odom = [0 0 0 0];
last_time = data(1).time;
nl = 1; nr = 1;

for i=1:length(data)

    if data(i).option == 0
        pose = movement_model(pose, [last_odom last_time], rNoise, data(i).time, wheeldistance, 0, Rn);
        last_odom = data(i).odom;
        last_time = data(i).time;
        xx(nr, :) = pose;
        nr = nr + 1;
    else
        for j=1:data(i).landmarksSeen
            landmark = new_landmark([data(i).landmark(j,3) data(i).landmark(j,2)], pose);          
            ll(nl, :) = [data(i).landmark(j,1) landmark];
            nl = nl + 1;
        end
    end
end

hold on;
plot(xx(:,1), xx(:,2));
for i=1:(nl-1)
    hold on;
    id = ll(i, 1);
    switch id
        case 0
            plot(ll(i,1), ll(i,2), '*b');
        case 1
            plot(ll(i,1), ll(i,2), '*g');
        case 2
            plot(ll(i,1), ll(i,2), '*y');
        case 3
            plot(ll(i,1), ll(i,2), '*m');            
        case 4
            plot(ll(i,1), ll(i,2), '*r');
        case 5
            plot(ll(i,1), ll(i,2), 'ob');
        case 6
            plot(ll(i,1), ll(i,2), 'og');
        case 7
            plot(ll(i,1), ll(i,2), '*g');  
        case 8
            plot(ll(i,1), ll(i,2), 'oy');
        case 9
            plot(ll(i,1), ll(i,2), 'om');  
        case 10
            plot(ll(i,1), ll(i,2), 'or');  
        case 11
            plot(ll(i,1), ll(i,2), '+b'); 
        case 12
            plot(ll(i,1), ll(i,2), '+g');             
    end
end
    
