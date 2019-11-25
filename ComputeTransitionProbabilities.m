function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
% find the treeList (number of tree * 1)
treeList = findTree(map);
num_tree = size(treeList,1);
m, n = size(map);
P = zeros(K,K,5);

for i = 1:K
    % Possible position can be reached from state i stateSpace(i) = (i,0/1)
    %                        I
    %                      F G H
    %                    A B i D E 
    %                      J K L
    %                        M
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        switch action
            case WEST
                
            case SOUTH
                
            case NORTH
                
            case EAST
                
            case HOVER
                
        end
    end
    
    posI_x, posI_y, state = stateSpace(i,:,:);
    posI_A = [posI_x, pos_y-2];
    idxI = n*posI_x + posI_y;
    idxA = n*posI_x + posI_y;
    % Compute number of trees between i and A: p 
    
end
    

end

function treeList = findTree(map)
% return an array treeList (number of tree * 1)
global TREE
treeList = [ ];
for i = 1:size(map,1)
    for j = 1:size(map,2)
        if map(i,j) == TREE
            treeList = [treeList;
                        i*n+j];
        end

    end
end
end

function idx = binarySearch(A, num)
% return the idx of a cell in the treeList
l = 1;
r = length(A);
   idx = 1;
   while l < r
      idx = 1 + floor((l + r - 1) / 2);
      if A(idx) > num
          r = idx - 1; 
      end
      if A(idx) < num
          l = idx + 1;
      end
      if A(idx) == num
          idx = -1; % Hitting the tree!
          return
      end
   end
   if l == r       
      idx = r; 
   end
   if A(idx) > num  % There is ne trees ahead of the cell!
     idx = 0;
   end
end
