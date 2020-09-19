function depth = depthPredict(ranges,angles)
% RANGEPREDICT: predict the range measurements for a robot operating
% in a known map.
% 
%   RANGE = RANGEPREDICT(ROBOTPOSE,MAP,ROBOTRAD,ANGLES) returns 
%   the expected range measurements for a robot operating in a known 
%   map. 
% 
%   INPUTS
%       range       	K-by-1 vector of ranges (meters) returned by rangePredict.m
%       angles      	K-by-1 vector of the angular positions of the range
%                   	sensor(s) in robot coordinates, where 0 points forward
% 
%   OUTPUTS
%       depth       	K-by-1 vector of depth (meters)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 2
%   Liu, Ertai
K = size(angles);
K = K(2);

depth = [];
for i = 1:K
    range = cos(angles(i))*ranges(i);
    depth = [depth,range];
end



    
end