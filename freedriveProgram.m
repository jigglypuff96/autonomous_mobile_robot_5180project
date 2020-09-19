function[dataStore] = freedriveProgram(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% freedriveProgram: example program to manually drive iRobot Create
% Reads data from sensors, sends robot commands, and saves a datalog.
% 
%   DATASTORE = freedriveProgram(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% 
%   INPUTS
%       CreatePort  Create port object (get from running CreatePiInit)
%       DepthPort   Depth port object (get from running CreatePiInit)
%       TagPort     Tag port object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   Lab #3
%   LASTNAME, FIRSTNAME 

% Set unspecified inputs
defaultMaxTime = 500;
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DepthPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 5
    maxTime = defaultMaxTime;
end

% Call up manual drive GUI
h = driveArrows(CreatePort,tagNum);

% declare datastore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;




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

Q = [0.01,0.001,0.002;0.001,0.01,0.002;0.002,0.002,0.015];      %process noise covariance
R1 = 0.7*eye(9);     %measurement noice covariance

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

%============================= end setup SPF ======================




% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);




noRobotCount = 0;
tic
while toc<maxTime
    % Read and Store Sensore Data
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DepthPort,TagPort,tagNum,noRobotCount,dataStore);

    
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
    
    
    
    
    %======================== END localization ==========================
    
    
    pause(0.1);
end


