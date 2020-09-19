function TF = isArrayInArrays(array, arrays)
TF = false;

for i = 1:size(arrays,1)
   if all(array == arrays(i,:))
       TF = true;
       return
   end

end


end