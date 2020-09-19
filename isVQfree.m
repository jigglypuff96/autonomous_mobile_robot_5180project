function isfree = isVQfree(obstacleVerts,q)

isfree = true;

for i = 1:length(obstacleVerts)
    xv = obstacleVerts{i}(:,1);
    yv = obstacleVerts{i}(:,2);
    [in] = inpolygon(q(1),q(2),xv,yv);
    if in
        isfree = false;
        return
    end
end


end