load('iterdata4.mat')

for i = 1:size(data,1)
    
    data(i).odom(1) = data(i).odom(1)/2;
    data(i).odom(3) = data(i).odom(3)/2;
    data(i).odom(2) = data(i).odom(2)*pi/180;
    data(i).odom(4) = data(i).odom(4)*pi/180;
    
end

iterdata = data;

save('iterdataPiso8_4.mat', 'iterdata')