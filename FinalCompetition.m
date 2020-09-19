function[dataStore] = FinalCompetition(CreatePort,DistPort,TagPort,tagNum,maxTime)
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
global Particles
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
                   'beacon', [],...
                   'estimatePose',[]);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.

%parameters
noRobotCount = 0;
%close all;


%% ===================back up bump setups ===============================
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

%% =========================SET UP SPF=====================================
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
    particle{6} = 1;
    particle{7} = i;
    Particles{i} = particle;
        
end


%Particles = repmat(Particles,1,2^size(optWalls,1));


t = -1;
isLost = 1;

%============================= end setup SPF ======================


%% ================= GLOBAL PATH PLANNER ================================
radius = 0.16;
elpsilon = 0.1;
points_to_go = [ECwaypoints;waypoints];
points_to_go = [points_to_go,zeros(size(points_to_go,1),1)];
isPathCurrent = 0;
path = [];
ratio = 1.85;
% ================= END GLOBAL PATH PLANNER ===============================


%% ================= logical parameters ===================================



tic
time_lost = tic;
isLostCount = 0;
isFirstLocalization = 1;
isFirstTimePlanning = 1;
isVote = 0;
voteFinished = 0;
%================= END logical parameters =================================

%% ===================== VOTE OPT WALLS =================================


wallToBeVoted = 0;
timeVoted = 0;
%%  ===================== First Loop Initialization ========================
Case = zeros(size(waypoints,1),1);
noisePF = [0,0,0]';
fgPF = @(initialPos,u) integrateOdom(initialPos,u,noisePF);
Xpf = [];  weightPF = [];numParticlesPF = 200;   sigma = 0.15;
for i = 1  : size(waypoints,1)
    Xpf = [Xpf;normrnd(waypoints(i,1),sigma,numParticlesPF,1),normrnd(waypoints(i,2),sigma,numParticlesPF,1),linspace(-pi,pi,numParticlesPF)'];
    weightPF = [weightPF;ones(numParticlesPF,1)/numParticlesPF];
end     



% ===================== END first loop initialization ===================

%% =========================== START WHILE LOOP =======================

while toc < maxTime
    % ========================== GENERAL PRINT =========================
%     display('new loop');
%     isVote
%     wallToBeVoted
%     timeVoted
    isLost
    % ========================== END GENERAL PRINT =====================
    if length(Particles) <=1
       isVoteFinished = 1; 
    end
    isbumped = flags(1);
    isbacked = flags(2);
    isturned = flags(3);
    time = toc;
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DistPort,TagPort,tagNum,noRobotCount,dataStore);
    
    
    %% ======================== localization =============================
    
    
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
    
    if isFirstLocalization == 1
        %% ====================== FIRST LOCALIZATION =====================
        Xv = [];
        fhPF = @(robotPose) hSPF(robotPose,Xv,map,sensorOrigin,angles,beaconLoc,beaconIndex);
        [Xpf_k,weight_k,weight_] = particleFilter(Xpf,weightPF,y,fgPF,fhPF,u,Q,R1,waypoints);
        
        [~,wMaxIdx] = max(weight_);    Case(wMaxIdx) = Case(wMaxIdx)+1; [~, maxCaseIdx] = max(Case); 
        [~,w_kMAxIdx] = max(weight_k((maxCaseIdx-1)*numParticlesPF+1:numParticlesPF*maxCaseIdx));
        %[~,w_kMAxIdx] = max(weight_k((wMaxIdx-1)*numParticlesPF+1:numParticlesPF*wMaxIdx));
        %X_kBAR = Xpf_k((wMaxIdx-1)*numParticlesPF+1:numParticlesPF*wMaxIdx,:);
        X_kBAR = Xpf_k((maxCaseIdx-1)*numParticlesPF+1:numParticlesPF*maxCaseIdx,:);
        Case
        Pose = X_kBAR(w_kMAxIdx,:)'; 
        Xx = Pose;
        Xpf = Xpf_k;  weightPF = weight_k;
        
        
       for i = 1:length(Particles)
            Particles{i}{1} = Xx;
            Particles{i}{3} = Pose;
        end
        
        % ====================== END FIRST LOCALIZATION ==================
    else
        %% ======================= NORMAL LOOP ==========================
    
        Particles = fakeParticleFilter(Particles,u,y,beaconIndex,beaconLoc,sensorOrigin,angles,Q,R1,sigma_f,f);
        
        [Pose,Pxx,Xx,voteFinished,Particles,timeVoted,map,optWalls,isPathCurrent] = sumParticles2(Particles,isVote,voteFinished,wallToBeVoted,timeVoted,map,optWalls,isPathCurrent);
        
        for i = 1:length(Particles)
            Particles{i}{1} = Xx;
            Particles{i}{2} = Pxx;
            Particles{i}{3} = Pose;
            
        end
        % ======================= END NORMAL LOOP =======================
    end
           
    
    %% ====================== PROCESS LOCALIZATION ========================
        
    x = Pose(1);
    y = Pose(2);
    th = Pose(3);
    

    
    [isVote, wallToBeVoted] = isVoteOptWalls(Pose,sensorOrigin,map,optWalls,isFirstLocalization);
    
    
    
    plot(Pose(1),Pose(2),'gx','MarkerSize',7);
    hold on;

    
    
    if(isInRange(Pose,map,radius))
        if isFirstLocalization == 1
            if  toc(time_lost)>25
                isLost = 0;
                isFirstLocalization = 0;
            end
        else
            if  toc(time_lost)>20
                isLost = 0;
            end
        end
        isLostCount = 0;
    else
        isLostCount = isLostCount+1;
        if isLostCount > 10
            isLost = 1;
            time_lost = tic;
            isLostCount = 0;
        end
    end

    

    
    %======================== END localization ==========================
    %% ======================== CHECK BUMP ===============================
    
    
    %read bump info
    
    if isLost == 1
        if any(dataStore.bump(end,2:end)~= 0)  %|| isDepthBumped(dataStore.rsdepth(end,3:end))~= 0
            
            isbumped = 1;
            isbacked = 0;
            isturned = 0;
            xbumped = x;
            ybumped = y;
            bumpMarker = tic;
            
        end
    else
        if any(dataStore.bump(end,2:end)~= 0)
            isbumped = 1;
            isbacked = 0;
            isturned = 0;
            xbumped = x;
            ybumped = y;
            bumpMarker = tic;
            isLost = 1;
            time_lost = tic;
        end

    end
    
    flags(1) = isbumped;
    flags(2) = isbacked;
    flags(3) = isturned;
    

    %======================== END CHECK BUMP ==========================
    
    %% ===================== Control ====================================

    if(isLost == 1 && isbumped ~= 1)
        cmdVd = 0;
        cmdWd = 0.1;
    elseif(isbumped == 1)
        
        [cmdVd, cmdWd,xbumped,ybumped,flags,bumpMarker] = backupBumpController(dataStore,x,y,th,xbumped,ybumped,flags,bumpMarker);
        time_lost = tic;
    else
        if isPathCurrent ==0
            startpoint = Pose(1:2)';
            SetFwdVelAngVelCreate(CreatePort, 0,0);
            isbumped = 0;
            isbacked = 0;
            isturned = 0;
            
            points_to_go(points_to_go(:,3) == 1,:) = [];
            figure 
            map
            points_to_go
            if isFirstTimePlanning == 1
                isFirstTimePlanning = 0;
                BeepRoomba(CreatePort);
                MDL = KDTreeSearcher(points_to_go(:,1:2));
                Idx = knnsearch(MDL,startpoint,'K',1);
                points_to_go(Idx,:) = [];
            end
            Radius = radius*ratio;
            for k = 1:10
                
                obstacles = wall2polygon(map,Radius);
                path = globalpathplanning_RP(map,obstacles,startpoint,points_to_go(:,1:2),radius);
                if size(path,1) <= size(points_to_go,1)
                    ratio = ratio - 0.02;
                    Radius = radius*ratio;
                    ratio
                else
                    break
                end
            end
            optWallIndex = optWallIsecPath(path,optWalls);

            nextPointIdx = min(find(path(:,4)==0));  nextPoint = path(nextPointIdx,1:2);
            isPathCurrent=1;
        end
        
        [cmdVd,cmdWd] = gotopointcontroller(path,Pose,elpsilon);      
        
    end
    %======================== END Control ==========================
    
    %% ====================== SEND VEL CMD ================================= 
    
    
    maxV = 0.1;
    wheel2Center = 0.13;
    [cmdV,cmdW] = limitCmds(cmdVd,cmdWd,maxV,wheel2Center);

    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0,0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    % ====================   END SEND VEL CMD =============================
    %% ======================== If  arrived  ===============================
    
    
    
    %isApproachedKeyPoint
    if isPathCurrent == 1 
        if (Pose(1)-nextPoint(1))^2 + (Pose(2)-nextPoint(2))^2 <=0.1^2
            voteFinished = 0;
            timeVoted = 0;
            path(nextPointIdx,4) = 1;
            % if is key point
            if path(nextPointIdx,3) == 1
                BeepRoomba(CreatePort);
            end
            if path(nextPointIdx,3) == 1
                points_to_go(sum(points_to_go(:,1:2) - nextPoint,2) == 0,3) = 1;
            end
            %end if key point
%             if length(optWallIndex) ~= 0
%                 if optWallIndex(1,2) == nextPointIdx
%                     nullOptWall = optWalls(optWallIndex(1,3),:);
%                 end
%             end
            
            
            nextPointIdx = min(find(path(:,4)==0));  nextPoint = path(nextPointIdx,1:2);
            if length(nextPoint) == 0
                SetFwdVelAngVelCreate(CreatePort, 0,0 );
                display('HAHAHAHAHAHAHAHAHAHAHAHA');
                figure
                for ipp = 1:size(map,1)
                    h1 = plot([map(ipp,1),map(ipp,3)],[map(ipp,2),map(ipp,4)],'k');
                    hold on
                end
                h2 = scatter(dataStore.estimatePose(:,1),dataStore.estimatePose(:,2),'gx');
                hold on
                
                for ipp = 1:size(optWalls,1)
                    h3 = plot([optWalls(ipp,1),optWalls(ipp,3)],[optWalls(ipp,2),optWalls(ipp,4)],'r');
                    hold on
                end
                xlabel('x (m)')
                ylabel('y (m)')
                title('Final Map & Trajectory ')
                if size(optWalls,1)~=0
                    legend([h1 h2 h3],'Final wall','trajectory','undecided opt wall')
                    return
                else
                    legend([h1 h2],'Final wall','trajectory')
                    return
                end
            end
           
        end

    end
    
    
    %% ======================== PLOT & END PROGRAM ======================
    if ~isempty(dataStore.truthPose)
        plot(dataStore.truthPose(:,2),dataStore.truthPose(:,3),'r');
    end
    dataStore.estimatePose = [dataStore.estimatePose; x y th];
    pause(0.5);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(CreatePort, 0,0 );
%% 
