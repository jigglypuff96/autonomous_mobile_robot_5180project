function [Particles] = fakeParticleFilter(Particles,u,y,beaconIndex,beaconLoc,sensorOrigin,angles,Q,R1,sigma_f,f)
%particles: particle = particles{i};
%particle{1} = Xx, particle{2} = Pxx patticle{3} = pose,  particle{4} =
%truemap particle{5} = weight particle {6} = score
nx = 3;
nw = 3;
nv1 = 9;
nv2 = length(beaconIndex)*2;
R2 = 0.01*eye(nv2);
R = [R1,zeros(nv1,nv2);zeros(nv2,nv1),R2];
nv = nv1+nv2;
na = nx+nw+nv;

Wm = (sigma_f^2-na)/(sigma_f^2);
Wc = ((sigma_f^2-na)/(sigma_f^2)+3-(sigma_f^2/na));

for i = 1:2*na
    Wm = [Wm,1/(2*sigma_f^2)];
    Wc = [Wc,1/(2*sigma_f^2)];
end

m = length(Particles);

for i = 1:m
    particle = Particles{i};
    Xx = particle{1};
    Pxx = particle{2};
    map = particle{4};
    Particlescore = particle{6};

    P = [[Pxx, zeros(nx,nw), zeros(nx,nv)];[zeros(nw,nx),   Q, zeros(nw,nv)];[zeros(nv,nx),   zeros(nv,nw), R]];
    h = @(robotPose,Xv) hSPF(robotPose,Xv,map,sensorOrigin,angles,beaconLoc,beaconIndex);
    xhata = [Xx;zeros(nw,1);zeros(nv,1)];
    X = [xhata, xhata*ones(1,na)-sigma_f*P^0.5, xhata*ones(1,na)+sigma_f*P^0.5];
    [X,P,score_walls] = SPF(X,P,Wm,Wc,u,y,sigma_f,nx,nw,nv,f,h);
    Pxx = P(1:3,1:3);
    Xx = X(1:nx,1);
    Pose = zeros(nx,1);
    for j = 1:2*na+1
        Pose = Pose+X(1:nx,j)*Wm(j);
    end
    %score = trace(Pxx);
    score = score_walls;
    Pxx(Pxx<0) = 0;
    if trace(Pxx)> 1E-7
        weight = 1/trace(Pxx);
    else
        weight = 1E7;
    end

    particle{1} = Xx;
    particle{2} = Pxx;
    particle{3} = Pose;
    particle{4} = map;
    particle{5} = 1/score;
    particle{6} = Particlescore;
    particle{8} = 1/score_walls;
    Particles{i} = particle;
    
end


    


end