function [Particles,map,optWalls] = changeParticles(Particles,optWall,isInclude,map,optWalls)


if isInclude
    i = 1;
    while i <= length(Particles)
        particleWall =  Particles{i}{4};
        isContained =0;
        for j = 1:size(particleWall,1)
            if all(particleWall(j,:) == optWall)
                isContained = 1;
                break
            end
        end
        if isContained == 0
            Particles(i) = [];
            i = i - 1;
        end
        i = i + 1;
    end
    
    map = [map;optWall];
    i = 1;
    while i <= length(optWalls)
        if all(optWalls(i,:) == optWall)
            optWalls(i,:) = [];
            i = i - 1;
            break
        end
        i = i + 1;
    end
    
    
else
    i = 1;
    while i <= length(Particles)
        particleWall =  Particles{i}{4};
        for j = 1:size(particleWall,1)
            if all(particleWall(j,:) == optWall)
                Particles(i) = [];
                i = i - 1;
                break
            end
        end
        i = i + 1;
    end
    i = 1;
    while i <= length(optWalls)
        if all(optWalls(i,:) == optWall)
            optWalls(i,:) = [];
            i = i - 1;
            break
        end
        i = i + 1;
    end
end


display(' ============ Particle changed: ==============')
optWall
isInclude
for i = 1:length(Particles)
    Particles{i}{7}
end

display('==============================================')

end