function pt = detSample(boundary,V,n)
x_min = boundary(1); x_max = boundary(2); y_min = boundary(3); y_max = boundary(4);
S = (x_max - x_min)*(y_max-y_min);
lambda = n^0.35;
R = (S*(n-lambda)/pi)^0.5/n;

if ~isempty(V)    
a = V(1,:); x=a(1); y=a(2);
while disC([x,y],V)<=R
    x = rand(1)*(x_max-x_min)+x_min;
    y = rand(1)*(y_max-y_min)+y_min;
end
pt = [x,y];
else
    x = rand(1)*(x_max-x_min)+x_min;
    y = rand(1)*(y_max-y_min)+y_min;
    pt= [x,y];
end

end


function D = disC(pt,pts)
[m,~] = size(pts);
D = 10000;
for i = 1:m
    d = dis(pt,pts(i,:));
    if D>d
        D=d;
    end
end
end

function d = dis(pt1,pt2)
d = ((pt1(1)-pt2(1))^2+(pt1(2)-pt2(2))^2)^0.5;
end