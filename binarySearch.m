function idx = binarySearch(A, num, Issearch)
% return the idx of a cell in the treeList
l = 1;
r = length(A);
   while l <= r
      idx = floor((l + r) / 2);
      if A(idx) > num
          r = idx - 1; 
      end
      if A(idx) < num
          l = idx + 1;
      end
      if A(idx) == num
          if Issearch
              idx = -1; % Hitting the tree!
          end
          return
      end
   end
   idx = l;
end