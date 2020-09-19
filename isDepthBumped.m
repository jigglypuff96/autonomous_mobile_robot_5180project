function TF = isDepthBumped(y)
y
minimumDistance = 0.2;
TF = 0;

for i = 3:length(y)-3
    if y(i) < minimumDistance && y(i) > 0.05
        TF = 1;
        break;
    end
    
end