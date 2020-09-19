function  [V, E,E_index,ifMerge,obstacles] = buildRRT(map,startPt, goalPt, radius)


x_max = max(max(map(:,1),max(map(:,3))));
x_min = min(min(map(:,1),min(map(:,3))));
y_max = max(max(map(:,2),max(map(:,4))));
y_min = min(min(map(:,2),min(map(:,4))));
obstacles = wall2polygon(map,radius);
N = 0; n = 100000; stepsize = min(x_max-x_min,y_max-y_min)/5;
scatter(startPt(1),startPt(2),'b');hold on;  scatter(goalPt(1),goalPt(2),'b');hold on;
[~,ob_size] = size(obstacles);
V = startPt;  E = []; E_index = [];  ifMerge = false;
while N<n
    x_rand = rand(1)*(x_max-x_min)+x_min;
    y_rand = rand(1)*(y_max-y_min)+y_min;
    q_rand = [x_rand,y_rand];
    
    Mdl = KDTreeSearcher(V);
    Idx = knnsearch(Mdl,q_rand,'K',1);
    q_near = V(Idx,:);
    
    q_new = (q_rand-q_near)/norm(q_rand-q_near)*stepsize + q_near;
    TF = false;
    for i =1:ob_size
        ob = obstacles{i};
        [in,on] = inpolygon(q_new(1),q_new(2),ob(:,1),ob(:,2));
        tf = isIntersect([q_new,q_near], obstacles);
        if in||on||tf
            TF = true;
            break
        end
    end
    if ~TF
        V = [V;q_new]; a = [q_new;q_near];% plot(a(:,1),a(:,2),'r'); hold on
        E = [E;[q_new,q_near]];   E = [E;[q_near,q_new]];   q_new_index = size(V,1);
        q_near_index = find(sum(V-q_near,2)==0);
        E_index = [E_index; q_new_index, q_near_index]; E_index = [E_index; q_near_index, q_new_index];   
        N = N+1;
        tf_f = isIntersect([q_new,goalPt], obstacles);
        if ~tf_f
            a = [q_new;goalPt];
            V = [V;goalPt]; E = [E;[q_new,goalPt]];E = [E;[goalPt,q_new]]; 
            E_index = [E_index; size(V,1)-1,size(V,1) ];  E_index = [E_index; size(V,1),size(V,1)-1 ];
            %plot(a(:,1),a(:,2),'r'); hold on
            ifMerge = true;
            break
        end
    end
end

