%bag = rosbag('2018-11-29-10-44-26.bag');
%pose = select(bag,'topic','/vrpn_client_node/pioneer/pose');
%msgStructs = readMessages(pose);

%x = cellfun(@(m) double(m.Pose.Position.X),msgStructs);
%y = cellfun(@(m) double(m.Pose.Position.Y),msgStructs);

figure();hold on;
plot(x(1),y(1),'or')
plot(x(2:3000), y(2:3000))