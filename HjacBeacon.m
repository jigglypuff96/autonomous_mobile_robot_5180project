function Hs = HjacBeacon(robotPose,beaconLoc,numBeacon)
% HjacGPS: return H matrix corresponding to GPS measurement
% 
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns 
%   the expected range measurements for a robot operating in a known 
%   map. 
% 
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%
%   OUTPUTS
%       H       		H matrix corresponding to GPS measurement
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Liu, Ertai
x = robotPose(1);
y = robotPose(2);
th = robotPose(3);
n = length(numBeacon);
for i = 1:n
    if numBeacon(i) <= 6
        numBeacon(i) = numBeacon(i)+1;
    end
end
Hs = [];
for i = 1:n
    beaconLocation = beaconLoc(numBeacon(i),2:3);
    xg = beaconLocation(1);
    yg = beaconLocation(2);
    H = [[ -cos(th), -sin(th), sin(th)*(x - xg) - cos(th)*(y - yg)];
        [  sin(th), -cos(th), cos(th)*(x - xg) + sin(th)*(y - yg)]];
    Hs = [Hs;H];
end
    
end