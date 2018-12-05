for i=1:size(data,1) 
    data(i).odom = [data(i).odom(1) data(i).odom(2)*pi/180 data(i).odom(3) data(i).odom(4)*pi/180]; 
end