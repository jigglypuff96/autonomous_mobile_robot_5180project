function isfree = isEQfree(V,obstacleVerts,one,two,map,specialObIdx)
if ~exist('specialObIdx','var') &&  ~exist('map','var')
    
    isfree = true;
    x1 = V(one,1);
    y1 = V(one,2);
    x2 = V(two,1);
    y2 = V(two,2);
    for i = 1:length(obstacleVerts)
        for j = 1:size(obstacleVerts{i},1)-1
            x3 = obstacleVerts{i}(j,1);
            y3 = obstacleVerts{i}(j,2);
            x4 = obstacleVerts{i}(j+1,1);
            y4 = obstacleVerts{i}(j+1,2);
            [isect,~,~,ua]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            if isInside(x1,y1,x2,y2,obstacleVerts) || (isect && ((abs(ua-0) > 0.00001) && (abs(ua-1) > 0.00001)))
                isfree = false;
                return
            end
        end
    end
else
    
    isfree = true;
    x1 = V(one,1);
    y1 = V(one,2);
    x2 = V(two,1);
    y2 = V(two,2);
    obstacleVerts(specialObIdx) = [];  iSect = false;
    for i = 1:length(obstacleVerts)
        for j = 1:size(obstacleVerts{i},1)-1
            x3 = obstacleVerts{i}(j,1);
            y3 = obstacleVerts{i}(j,2);
            x4 = obstacleVerts{i}(j+1,1);
            y4 = obstacleVerts{i}(j+1,2);
            [isect,~,~,ua]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
            for k = 1:length(specialObIdx) 
                Wall = BloatWall(map(specialObIdx(k),:));
            [iSSect,~,~,Ua]= intersectPoint(x1,y1,x2,y2,Wall(1),Wall(2),Wall(3),Wall(4));
            iSect = iSect || (iSSect && ((abs(Ua-0) > 0.00001) && (abs(Ua-1) > 0.00001))) ;
            end
            if isInside(x1,y1,x2,y2,obstacleVerts) || (isect && ((abs(ua-0) > 0.00001) && (abs(ua-1) > 0.00001))) || iSect
                isfree = false;
                return
                
            end
        end
    end
end
end

function returnVal = isInside(x1,y1,x2,y2,obstacleVerts)
returnVal = false;
xq = (x1+x2)/2;
yq = (y1+y2)/2;
for i = 1:length(obstacleVerts)
    xv = obstacleVerts{i}(:,1);
    yv = obstacleVerts{i}(:,2);
    [in,on] = inpolygon(xq,yq,xv,yv);
    if in && ~on
        returnVal = true;
    end
end
end

function Wall = BloatWall(wall)
iniPt = [wall(1),wall(2)];
    endPt = [wall(3),wall(4)];
    new1 = (endPt - iniPt)/norm(endPt - iniPt)*0.18 + endPt;
    new2 = (iniPt - endPt)/norm(endPt - iniPt)*0.18 + iniPt;
    Wall = [new1, new2];
end
