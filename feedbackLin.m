function[V,W] = feedbackLin(Vx,Vy,theta,sigma)
% feedbackLin: transfer Vx Vy to V and W
% 
%   INPUTS
%       Vx           velocity (m/s) in x direction
%       Vy           velocity (m/s) in y direction
%       theta        the angle that robot rotates
%       sigma        the turning speed
% 
%   OUTPUTS
%       V        forward velocity
%       W        angular velocity
% 
V = Vx*cos(theta) + Vy*sin(theta);
W = -1/sigma*Vx*sin(theta) + 1/sigma*Vy*cos(theta);