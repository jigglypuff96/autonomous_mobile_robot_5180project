function measure = hBeacon(robotPose,beaconLoc,beaconIndex,sensorOrigin)
% hGPS: predict the Beacon measurements for a robot with estimated position.
% 
% 
%   INPUTS
%       robotPose   	1x3 pose vector in global coordinates (x,y,theta)
%       robotCalibration [0.13,0]
%
%   OUTPUTS
%       [x,y]       	estimated beacon measurements
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Liu, Ertai
n = length(beaconIndex);

measure = [];
for i = 1:n
    beaconLocation = beaconLoc(beaconLoc(:,1)==beaconIndex(i),2:3);
    measure = [measure,global2robot(robotPose,beaconLocation)-sensorOrigin];
end

end