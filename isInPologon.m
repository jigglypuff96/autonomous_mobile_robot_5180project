function TF = isInPologon(line, obstacles)
[~,n] = size(obstacles); 
for i = 1:n
    ob = obstacles{i};
    [in,on] = inpolygon((line(1)+line(3))/2,(line(2)+line(4))/2,ob(:,1),ob(:,2));
    if in&&~on
        TF = true;
        return 
    end
end
TF = false;