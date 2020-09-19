function TF = isIntersect(line, obstacles)
x1 = line(1); y1 = line(2); x2 = line(3); y2 = line(4);
[~,n] = size(obstacles); 
for i = 1:n
    ob = obstacles{i};
    [p,~] = size(ob);
    for j = 1:p-1         
        x3 = ob(j,1); y3 = ob(j,2); x4 = ob(j+1,1); y4 = ob(j+1,2); 
        [isect,~,~,ua] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        if isect
            if ~(abs(ua-1)<0.001 || abs(ua)<0.001)
                TF = true;
                break
            end
        else
            TF = false;
        end
    end
    if TF
        break
    end
end
