function [X_k,w_k,w_] = particleFilter(X,w,y,g,h,u,Q,R1,waypoints)
%input     X: nx3
%          w: nx1
%          Z: 9x1  measurement
%          g: dynamics function
%          h: measurement prediction
%          u: control
%          u: control
%output    X_k: nx3
%          w: nx1
L = length(w); numParticles = L/size(waypoints,1); w_ = []; w_k = [];
X_k = []; X_bar = zeros(L,3); 

%% calculate weight   wi = wi*p(z|x)
nv1 = size(R1,1);
nv2 = length(y)-nv1;
R2 = 0.001*eye(nv2);
R = [R1,zeros(nv1,nv2);zeros(nv2,nv1),R2];


p = length(y); y_bar = zeros(L,p); 

toDelete = find(y == 0);  
if ~isempty(toDelete)
    y(toDelete) = [];  R(toDelete,:) = [];  R(:,toDelete) = []; 
end
for i = 1:L
    X_bar(i,:) = g(X(i,:),u)' + normrnd([0 0 0],[Q(1,1)^0.5,Q(2,2)^0.5,Q(3,3)^0.5],[1,3]);
    y_bar(i,:) = h(X_bar(i,:));
    if ~isempty(toDelete)
        aa = y_bar(i,:);
        aa(toDelete) = [];
    else
        aa = y_bar(i,:);
    end
    
%         w_k(i) = chi2pdf( norm(Z'-Z_bar), p)*w(i);
    if isempty(y)
        w_kk(i) = 1/numParticles;
    else
        w_kk(i) = mvnpdf(y,aa,R);
    end
end
w_kk = w_kk';
for i = 1:size(waypoints,1)
    w_bar = w_kk((i-1)*numParticles+1:i*numParticles);
    XX_bar = X_bar((i-1)*numParticles+1:i*numParticles,:);
    w_ = [w_; sum(w_bar)];
    if sum(w_bar)==0
        w_bar = ones(numParticles,1)/numParticles;
    else
        w_bar = w_bar/sum(w_bar);
    end
    
    c(1) = w_bar(1);   b = zeros(size(XX_bar,1),size(XX_bar,2));
    for j = 2:numParticles
        c(j) = c(j-1)+w_bar(j);
    end
    U(1) = rand; U(1) = U(1)/numParticles;
    j = 1;
    for k = 1:numParticles
        U(k) = U(1) + (k-1)/numParticles;
        while U(k)>c(j)
            j = j+1;
        end
        b(k,:) = XX_bar(j,:); a(k) = w_kk(j);
    end
    %yIdx = randsample(linspace(1,numParticles,numParticles),numParticles,true,w_bar');
    a = a/sum(a);
    X_k = [X_k;b]; w_k = [w_k ; a'];
end
    
% resample 
% c(1) = w_kk(1);
% for i = 2:L
%     c(i) = c(i-1)+w_kk(i);
% end
% U(1) = rand; U(1) = U(1)/L;
% i = 1;
% for j = 1:L
%     U(j) = U(1) + (j-1)/L;
%     while U(j)>c(i)
%         i = i+1;
%     end
%     X_k(j,:) = X_bar(i,:); w_k(j) = w_kk(i);
% end



end