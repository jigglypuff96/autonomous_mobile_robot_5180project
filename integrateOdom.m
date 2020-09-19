function X = integrateOdom(initialPos,u,Xw)
% integrateOdom function for MAE 4180/5180, Homework 2.  
% This function checks the student's "integrate odometry" implementation.
% 
%   INPUTS
%       initialPos       Initial/previous robot configuration,  3x1 [x y theta]'
%       u                [Distance,Angle]
% 
%   OUTPUTS
%       [x,y,th]    positions and angles calculation based on odometry
%                   data, each is 1xN
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Liu, Ertai

%initialization
x = [initialPos(1)];
y = [initialPos(2)];
theta = [initialPos(3)];
%integration

D = u(1);
A = u(2);
if A ==0
    x = x + cos(theta)*D;
    y = y + sin(theta)*D;
else
    x = x + (D*sin(theta+A) - D*sin(theta))/A;
    y = y - (D*cos(theta+A) - D*cos(theta))/A;
end
theta = theta+A;
X = [x,y,theta]'+Xw;



end