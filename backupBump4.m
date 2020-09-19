function[dataStore] = backupBump4(CreatePort,DistPort,TagPort,tagNum,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot back up and turn when bumped and saves a datalog.
% 
%   dataStore = TURNINPLACE(CreatePort,tagNum,maxTime) runs 
% 
%   INPUTStype
%       CreatePort  Create port object (get from running RoombaInit)
%       RSDepth_pts number of points to take in RealSense depth scan
%       RSDepth_hgt height at which to take RealSense Depth scan
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DistPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = 500;
elseif nargin < 5
    maxTime = 500;
end


% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.

%parameters
noRobotCount = 0;
%close all;


% ===================back up bump setups ============================
isbumped = 0;
isbacked = 0;
isturned = 0;
xbumped = 0;
ybumped = 0;
flags(1) = isbumped;
flags(2) = isbacked;
flags(3) = isturned;
bumpMarker = 0;

% ================= end back up bump setups ============================



%=========================SET UP SPF=====================================
load('compMap.mat');
figure;
for i = 1:size(map,1)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'r')
    
    hold on;
end
hold on

sigma_f = 1.5;

sensorOrigin = [0.16,0];
angles = linspace(27/180*pi,-27/180*pi,9);
% na = nx+nw+nv;
Pxx0 = eye(3)*1;  %initial P matrix

Q = 0.01*eye(3);      %process noise covariance
R1 = 0.25*eye(9);     %measurement noice covariance

Xx0 = [0,0,0]';
f = @integrateOdom;

numParticles = 2^size(optWalls,1);

possibleWalls = {};
possibleWalls{1} = map;
[numOptWalls,~] = size(optWalls);
for optI = 1: numOptWalls
    possibleCombination = nchoosek(1:numOptWalls,optI);
    [numCombination,~] = size(possibleCombination);
    for index1 = 1: numCombination
        optWallIndexNow = possibleCombination(index1,:);
        numOptWallAdded = length(optWallIndexNow);
        wallList = [];
        for index2 = 1:numOptWallAdded
            optNow = optWallIndexNow(index2);
            wallList = [wallList;optWalls(optNow,:)];
        end
        RealWallList = [map;wallList];
        possibleWalls{end+1} = RealWallList;
    end
end

numMapChoice = length(possibleWalls);
    
for i = 1:numParticles
    
    truemap = possibleWalls{i};
    
    %truemap = [map;[0,0,0,-0.7]];
    
    %to be finished
    particle{1} = Xx0;
    particle{2} = Pxx0;
    particle{4} = truemap;
    particle{6} = i;
    Particles{i} = particle;
    
    
end


%Particles = repmat(Particles,1,2^size(optWalls,1));


t = -1;
isLost = 1;

%============================= end setup SPF ======================





% ================= GLOBAL PATH PLANNER ================================
radius = 0.16*1.1;
elpsilon = 0.1;
points_to_go = [ECwaypoints;waypoints];
isPathCurrent = 0;
path = [];
% ================= END GLOBAL PATH PLANNER ===============================


tic
time_lost = tic;
while toc < maxTime
    isbumped = flags(1);
    isbacked = flags(2);
    isturned = flags(3);
    time = toc;
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    %======================== localization =============================
    y = dataStore.rsdepth(end,3:end);
    beaconIndex = [];
    if (length(dataStore.beacon) ~= 0 && (dataStore.beacon(end,1) ~= t))
        y = [y,dataStore.beacon(end,4:5)];
        beaconIndex = [beaconIndex,dataStore.beacon(end,3)];
        t = dataStore.beacon(end,1);
        for i = 1:size(dataStore.beacon,1)-1
            if dataStore.beacon(end-i,1) == t
                y = [y,dataStore.beacon(end-i,4:5)];
                beaconIndex = [beaconIndex,dataStore.beacon(end-i,3)];
            else
                break;
            end
        end
    end
    
    u = dataStore.odometry(end,2:3);
    Particles = fakeParticleFilter(Particles,u,y,beaconIndex,beaconLoc,sensorOrigin,angles,Q,R1,sigma_f,f);
    
    [Pose,Pxx] = sumParticles2(Particles);
    score = sum(sum(Pxx));
    plot(Pose(1),Pose(2),'gx','MarkerSize',7);
    hold on;
    
    
    if(isInRange(Pose,map,radius))
        if  toc(time_lost)>30
            isLost = 0;
        end
    else
        time_lost = tic;
    end
    
    x = Pose(1);
    y = Pose(2);
    th = Pose(3);
    
    
    %======================== END localization ==========================
    

    
    %read bump info
    for i = 2:size(dataStore.bump,2)  %i = 1 is the time info
        if dataStore.bump(end,i)~= 0  || isDepthBumped(dataStore.rsdepth(end,3:end))~= 0

            isbumped = 1;
            isbacked = 0;
            isturned = 0;
            xbumped = x;
            ybumped = y;
            bumpMarker = tic;
            
        end
    end
    
    flags(1) = isbumped;
    flags(2) = isbacked;
    flags(3) = isturned;

    
    %===================== Control ======================
    
    
%     x = dataStore.truthPose(end,2);
%     y = dataStore.truthPose(end,3);
%     th = dataStore.truthPose(end,4);

    % CONTROL FUNCTION (send robot commands)
    
    
    
    isbumped
    isLost
    if(isbumped == 1 || isLost == 1)
        
        [cmdVd, cmdWd,xbumped,ybumped,flags,bumpMarker] = backupBumpController(dataStore,x,y,th,xbumped,ybumped,flags,bumpMarker);
    else
        if isPathCurrent ==0
            startpoint = Pose(1:2)';
            SetFwdVelAngVelCreate(CreatePort, 0,0);
            isbumped = 0;
            isbacked = 0;
            isturned = 0;
            path = globalpathplanning2([map;0,0,0,-0.7],startpoint,points_to_go,radius);
            isPathCurrent=1;
        end
    
        [cmdVd,cmdWd] = gotopointcontroller(path,Pose,elpsilon);      
        
    end
    %======================== END Control ==========================
    
    
    
    maxV = 0.1;
    wheel2Center = 0.13;
    [cmdV,cmdW] = limitCmds(cmdVd,cmdWd,maxV,wheel2Center)
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    
    
    
    
    % ======================== If  arrived  ===============================
    if isPathCurrent == 1
        nextPointIdx = min(find(path(:,4)==0));  nextPoint = path(nextPointIdx,1:2);
        if (Pose(1)-nextPoint(1))^2 + (Pose(2)-nextPoint(2))^2 <=0.1^2
            path(nextPointIdx,4) = 1;
            if path(nextPointIdx,3) == 1
                BeepRoomba(CreatePort);
            end
        end
    end
    
    
    
    plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'r');
    
deltaT = toc - time
pause(0.5)
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );