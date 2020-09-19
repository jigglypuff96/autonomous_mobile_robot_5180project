function [cmdVd, cmdWd,xbumped,ybumped,flags,bumpMarker] = backupBumpController(dataStore,x,y,th,xbumped,ybumped,flags,bumpMarker)
isbumped = flags(1);
isbacked = flags(2);
isturned = flags(3);
backlimit = 0.2;



%record backed distance and turned angle if bumped
backed = sqrt((x-xbumped)^2+(y-ybumped)^2)


%if bumped
if isbumped == 1
    %if backed > backlimit
    if (toc(bumpMarker)>=2)
        isbacked = 1;
    end
    
    
    if isbacked == 0
        cmdVd = -0.05;
        cmdWd = 0;
%     elseif isturned == 0
%         cmdVd = 0;
%         cmdWd = -0.5;
%         toc(bumpMarker)
%         if (toc(bumpMarker)>=3)
%             cmdWd = 0;
%             isturned = 1;
%             isbumped = 0;
%         end
    else
        %bumped situation dealed. Reset all flags
        cmdVd = 0;
        cmdWd = 0;
        isbumped = 0;
        isbacked = 0;
        isturned = 0;
    end
    
    
else
    cmdVd = 0.05;
    cmdWd = 0;
end



flags(1) = isbumped;
flags(2) = isbacked;
flags(3) = isturned;

end