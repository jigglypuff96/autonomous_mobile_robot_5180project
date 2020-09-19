function TF = isInRange(Pose,map,radius)
xmin = min(min(map(:,1)),min(map(:,3)));
xmax = max(max(map(:,1)),max(map(:,3)));
ymin = min(min(map(:,2)),min(map(:,4)));
ymax = max(max(map(:,2)),max(map(:,4)));
x = Pose(1);
y = Pose(2);
obstacles = wall2polygon(map,radius*0.9);
[~,ob_size] = size(obstacles);
for i =1:ob_size
        ob = obstacles{i};
        [in,on] = inpolygon(x,y,ob(:,1),ob(:,2));
        
        if in
            TF = false;
            return
        end
end

TF = x>xmin && x<xmax && y<ymax && y>ymin;
end