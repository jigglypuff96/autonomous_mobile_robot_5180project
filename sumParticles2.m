function [Pose,Pxx,Xx,voteFinished,Particles,timeVoted,map,optWalls,isPathCurrent] = sumParticles2(Particles,isVote,voteFinished,votedPoly,timeVoted,map,optWalls,isPathCurrent)
if isVote == 0
   for i = 1:length(Particles)
      Particles{i}{6} = 0; 
      timeVoted = 0;
   end
end


if length(votedPoly) ~= 4 || voteFinished == 1
   isVote = 0; 
end
weights = [];
weights_walls = [];
for i = 1:length(Particles)
    weights = [weights,Particles{i}{5}];

end
for i = 1:length(Particles)
    weights_walls = [weights_walls,Particles{i}{8}];

end
%weights;
[maxWeights,index] = max(weights);

Pose = Particles{index}{3};
Pxx = Particles{index}{2};
Xx = Particles{index}{1};


if isVote
    timeVoted = timeVoted + 1;
    scores = [];
    for i = 1:length(Particles)
%         if weights(i) == maxWeights
%             Particles{i}{6} =  Particles{i}{6} + 1;
%             
%         end
        Particles{i}{6} =  Particles{i}{6} + weights_walls(i)/sum(weights_walls);
        scores = [scores,Particles{i}{6}];
        
    end
    scores
    for i = 1:length(scores)
       if isnan(scores(i))
           scores = zeros(1,length(scores));
           for j = 1:length(Particles)
              Particles{j}{6} = 0; 
           end
           break
       end
    end
    
    
    
    std(scores)
    if std(scores)>0.4 || timeVoted >3
    %if timeVoted >10
        wallDetermined = 0;
        for i = 1:length(Particles)
            if isArrayInArrays(votedPoly, Particles{i}{4})
                wallDetermined = wallDetermined+Particles{i}{6};
            else
                wallDetermined = wallDetermined-Particles{i}{6};
            end
            
        end
        wallDetermined
        if wallDetermined>0
            [Particles,map,optWalls] = changeParticles(Particles,votedPoly,true,map,optWalls);
            isPathCurrent = 0;
        elseif wallDetermined<0
            [Particles,map,optWalls] = changeParticles(Particles,votedPoly,false,map,optWalls);
        end
        
        for i = 1:length(Particles)
            Particles{i}{6} =  0;
            
        end
        voteFinished = 1;
        timeVoted = 0;
        
    end
end



%weightSum = sum(weights);
%Pose = Pose.*weights';

%Pose = sum(Pose)/weightSum;
