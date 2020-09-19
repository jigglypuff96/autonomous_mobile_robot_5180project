function depth = hDepth(robotPose,map,sensorOrigin,angles)
% hDepth: predict the depth measurements for a robot operating
% in a known map.
% 
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
%       depth       	1-by-K vector of depth (meters)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Liu, Ertai
ranges = rangePredict(robotPose,map,sensorOrigin,angles);
depth = depthPredict(ranges,angles);
    
end