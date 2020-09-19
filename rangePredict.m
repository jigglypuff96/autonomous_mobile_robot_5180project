function range = rangePredict(robotPose,map,sensorOrigin,angles)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
% 
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns 
%   the expected range measurements for a robot operating in a known 
%   map. 
% 
%   INPUTS
%       robotPose   	1-by-3 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in 
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
% 
%   OUTPUTS
%       range       	K-by-1 vector of ranges (meters)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Liu, Ertai

K = size(angles);
K = K(2);
N = size(map);
N = N(1);
maxR = 10;
range = [];

for i = 1:K
    angle = angles(i);
    endPos = [maxR*cos(angle)+sensorOrigin(1),maxR*sin(angle)+sensorOrigin(2)]; %line 1 end point in robot frame
    endPos = robot2global(robotPose,endPos);                                    %line 1 end point in initial frame
    startPos = robot2global(robotPose,sensorOrigin);                            %line 1 start point in initial frame
    minD = maxR;                       %maximum range
    for j = 1:N
        wall = map(j,:);
        x1 = startPos(1);
        y1 = startPos(2);
        x2 = endPos(1);
        y2 = endPos(2);
        x3 = wall(1);
        y3 = wall(2);
        x4 = wall(3);
        y4 = wall(4);
        [ISECT,X,Y,UA] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        if ISECT == false
            d = maxR;
        else
            d = UA*maxR;
        end
        
        if d < minD
            minD = d;
        end 
    end
    range = [range,minD];
    
end


