function obstacles = optWall2polygon(map,radius)
[size_map,~] = size(map);
for i = 1: size(map,1)
    iniPt = [map(i,1),map(i,2)];
    endPt = [map(i,3),map(i,4)];
    new1 = (endPt - iniPt)/norm(endPt - iniPt)*radius + endPt;
    new2 = (iniPt - endPt)/norm(endPt - iniPt)*radius + iniPt;
    vec_ver1 = cross([0,0,1],[(endPt - iniPt)/norm(endPt - iniPt),0]);
    vec_ver2 = cross([0,0,1],[(iniPt - endPt)/norm(endPt - iniPt),0]);
    e1 = iniPt+vec_ver1(1:2)*radius;  
    e2 = iniPt-vec_ver1(1:2)*radius;
    e3 = endPt+vec_ver2(1:2)*radius;  
    e4 = endPt-vec_ver2(1:2)*radius;
    obstacles{i} = [e1;e2;e3;e4;e1];
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k')
    hold on
    plot(obstacles{i}(:,1),obstacles{i}(:,2))
    hold on
end

