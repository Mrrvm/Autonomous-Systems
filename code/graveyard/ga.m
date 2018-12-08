load('dataCorredorSquare.mat')

s = size(data,2);

d = data(1);

for i = 2:s

    if data(i).option == 0
        d = [d; data(i)];    
    end
end

load(fullfile('data','iterdatacorredorsquare.mat'))