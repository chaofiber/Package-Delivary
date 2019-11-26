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
global L
global BASE_STATE_INDEX

%% INITILIZATION
% find the treeList (number of tree * 1)
[treeList, shooterList] = findTree(map);
Prob_Survive = Survive(shooterList);
num_tree = size(treeList,1);
global m n
[m, n] = size(map);

P = zeros(K,K,L);
P_IMNORMAL_TO_BASE = zeros(K,L);
%% COMPUTE ODD SUB TRANSITION MATRIX FIRST
for i = 1:2:K
    % Possible position can be reached from state i stateSpace(i) = (i,0/1)
    %                        I
    %                      F G H
    %                    A B i D E 
    %                      J K L
    %                        M
    
    pos_state_i = stateSpace(i,:,:);
    pos_i = pos_state_i(1:2);
    pos_A = [pos_i(1), pos_i(2)-2];
    idxI = n*pos_i(1) + pos_i(2);
    idxA = n*pos_A(1) + pos_A(2);
    p = binarySearch(treeList, idxI, false) - binarySearch(treeList, idxA, false);
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        
        switch action
            case WEST
                % Check if this action is allowed ( NOT HITTING A TREE AND
                % NOT OUT OF BORDER !!!)
                % ppos: the cell the drone is supposed to be in if no wind
                % happens
                ppos = pos_i + [-1, 0];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    % Search possible states after WEST action and the wind
                    for j = [ todo here ] % possible reachable state indexs]
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j
                            case pos_i + [-1, 0]
                                P_temp = (1 - P_WIND); 
                                P(i, j, action) = p_temp*Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                            case pos_i
                                P_temp = P_WIND * 0.25;
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                            case {pos_i + [-1, 1], pos_i + [-1,-1], pos_i + [-2, 0]}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    P(i ,j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                    P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end
                
            case SOUTH
                ppos = pos_i + [0, -1];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    % Search possible states after WEST action and the wind
                    for j = [ todo here ] % possible reachable state indexs]
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j
                            case pos_i + [0, -1]
                                P(i, j, action) = (1 - P_WIND); 
                            case pos_i
                                P(i, j, action) = P_WIND * 0.25;
                            case {pos_i + [-1, -1], pos_i + [1,-1], pos_i + [0, -2]}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P(i, j, action) = P_WIND * 0.25;
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end    
                
            case NORTH
                ppos = pos_i + [0, 1];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    % Search possible states after WEST action and the wind
                    for j = [ todo here ] % possible reachable state indexs]
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j
                            case pos_i + [0, 1]
                                P(i, j, action) = (1 - P_WIND); 
                            case pos_i
                                P(i, j, action) = P_WIND * 0.25;
                            case {pos_i + [-1, 1], pos_i + [1,1], pos_i + [0, 2]}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P(i, j, action) = P_WIND * 0.25;
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end
                
            case EAST
                ppos = pos_i + [1, 0];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    % Search possible states after WEST action and the wind
                    for j = [ todo here ] % possible reachable state indexs]
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j
                            case pos_i + [1, 0]
                                P(i, j, action) = (1 - P_WIND); 
                            case pos_i
                                P(i, j, action) = P_WIND * 0.25;
                            case {pos_i + [1, 1], pos_i + [1,-1], pos_i + [2, 0]}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P(i, j, action) = P_WIND * 0.25;
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end
                
            case HOVER
                ppos = pos_i + [0, 0];
                if binarySearch(treeList,n*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    % Search possible states after WEST action and the wind
                    for j = [ todo here ] % possible reachable state indexs
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j
                            case pos_i + [0, 0]
                                P(i, j, action) = (1 - P_WIND); 
                            case {pos_i + [0, 1], pos_i + [0,-1], pos_i + [1, 0], pos_i + [-1, 0]}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P(i, j, action) = P_WIND * 0.25;
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end                
        end
    end

    % Compute number of trees between i and A: p 
    
end

%% COPY TO THE EVEN INDEX SUB TRANSITION MATRIX
% TO DO
    

end

function [treeList, shooterList] = findTree(map)
% return a treeList matrix (nb_tree * 1)
global TREE SHOOTER
treeList = [ ];
shooterList = [ ];
for i = 1:size(map,1)
    for j = 1:size(map,2)
        if map(i,j) == TREE
            treeList = [treeList;
                        i*n+j];
        end
        if map(i,j) == SHOOTER
            shooterList = [shooterList;
                           i,j];
        end

    end
end
end

function idx = binarySearch(A, num, Issearch)
% return the idx of a cell in the treeList, for example, if the treeList is
% [3, 4, 12, 23, 45, 66], inserting a value 5 will output 3, inserting a
% value 2 will output 1, inserting a value 45 with Issearch=true will
% output -1, with Issearch=false will output 5.

% If set Issearch = true, return -1 for hitting the tree, otherwise, return
% the index
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

function out = OutOfBorder(width, height, pos)
% return true if point (x,y) is out of map border, otherwise return false
if pos(1)<1 || pos(1)>width || pos(2)<1 || pos(2)>height
    out = true;
else
    out = false;
end
end

function Prob_not_shooted = Survive(shooterlist)
% return the probability matrix (m*n), where each meaningful cell represents the probability
% of surviving the angry resident. We also give value 1 to the tree cells, but
% will never use them in the experiment.
global GAMMA R stateSpace m n
Prob_not_shooted = ones(m,n);
nb_shooter = size(shooterlist,1);
for j = 1:2:K
    pos = stateSpace(j,1:2);
    for i = 1:nb_shooter
        shooter = shooterlist(i);
        d = abs(pos(1) - shooter(1)) + abs(pos(2) - shooter(2));
        if d <= R
            Prob_not_shooted(pos(1), pos(2)) = Prob_not_shooted(pos(1), pos(2)) * (1 - (GAMMA/(1 + d)));
        end
    end
end
end
