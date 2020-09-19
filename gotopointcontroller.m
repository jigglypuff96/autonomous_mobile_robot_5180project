function [cmdV,cmdW] = gotopointcontroller(path,nowPose,elpsilon)
nowPoseX = nowPose(1);  nowPoseY = nowPose(2);  nowPosetheta = nowPose(3);

nextPointIdx = min(find(path(:,4)==0));
Vx = path(nextPointIdx,1) - nowPoseX;  Vy = path(nextPointIdx,2) - nowPoseY;

[cmdV,cmdW] = feedbackLin(Vx,Vy,nowPosetheta,elpsilon);