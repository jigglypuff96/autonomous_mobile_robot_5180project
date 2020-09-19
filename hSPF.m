function measure = hSPF(robotPose,Xv,map,sensorOrigin,angles,beaconLoc,beaconIndex)
robotPose = robotPose';
depth = hDepth(robotPose,map,sensorOrigin,angles);

beacon = hBeacon(robotPose,beaconLoc,beaconIndex,sensorOrigin);
if isempty(Xv)
   measure = [depth,beacon];
measure = measure'; 
else
measure = [depth,beacon]+Xv';
measure = measure';
end
end