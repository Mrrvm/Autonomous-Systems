function f = landmark_gen(xmin,xmax,ymin,ymax,n,randFlag)
    if randFlag
        f=[randi([xmin xmax],1,n); randi([ymin ymax],1,n)];
    else
        f=cloister(xmin,xmax,ymin,ymax,n);
    end
end
