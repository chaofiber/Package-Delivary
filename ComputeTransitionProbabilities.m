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
global m n
[m, n] = size(map);

P = zeros(K,K,5);

for i = 1:2:K
    % Possible position can be reached from state i stateSpace(i) = (i,0/1)
    %                        I
    %                      F G H
    %                    A B i D E 
    %                      J K L
    %                        M
    
    posI_x, posI_y, state = stateSpace(i,:,:);
    posA = [posI_x, pos_y-2];
    posE = [posI_x, pos_y+2];
    idxI = n*posI_x + posI_y;
    idxA = n*posI_x + posI_y;
    p = binarySearch(treeList, idxI, false) - binarySearch(treeList, idxA, false);
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        switch action
            case WEST
                % Check if this action is allowed
                ppos = [posI_x, posI_y-1];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1
                    
                end
                    
                    
            case SOUTH
                
            case NORTH
                
            case EAST
                
            case HOVER
                
        end
    end

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

function idx = binarySearch(A, num, Issearch)
% return the idx of a cell in the treeList
% If set Issearch = true, return -1 for hitting the tree, otherwise, return
% the index
l = 1;
r = length(A);
   idx = 1;
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
