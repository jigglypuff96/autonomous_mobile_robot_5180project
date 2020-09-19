function plotVerts(Verts)
for i = 1:length(Verts)
    for j = 1:size(Verts{i},1)-1
        plot([Verts{i}(j,1),Verts{i}(j+1,1)],[Verts{i}(j,2),Verts{i}(j+1,2)])
        hold on
        
    end


end