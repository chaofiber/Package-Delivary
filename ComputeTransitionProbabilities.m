function [P, P_IMNORMAL_TO_BASE] = ComputeTransitionProbabilities( stateSpace, map)
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
%global BASE_STATE_INDEX

%% INITILIZATION
% find the treeList (number of tree * 1)
global m n
[m, n] = size(map);
L = 5;
[treeList, shooterList] = findTree(map);
Prob_Survive = Survive(shooterList, stateSpace);
num_tree = size(treeList,1);
[x,y] = find(map==BASE);
BASE_STATE_INDEX = find(ismember(stateSpace,[x,y,0],'rows'));
[x,y] = find(map==PICK_UP);
PICK_UP_INDEX = find(ismember(stateSpace,[x,y,1],'rows'));

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
    pos_A = [pos_i(1)-2, pos_i(2)];
    pos_E = [pos_i(1)+2, pos_i(2)];
    idxI = n*pos_i(1) + pos_i(2);
    idxA = n*pos_A(1) + pos_A(2);
    idxE = n*pos_E(1) + pos_E(2);
    p = binarySearch(treeList, idxI, false) - binarySearch(treeList, idxA, false);
    q = binarySearch(treeList, idxE, false) - binarySearch(treeList, idxE, false);
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
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i ] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[n,1]'
                            case (pos_i + [-1, 0])*[n,1]'
                                P_temp = (1 - P_WIND); 
                                P(i, j, action) = P_temp*Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                            case pos_i*[n,1]'
                                P_temp = P_WIND * 0.25;
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                            case {(pos_i + [-1, 1])*[n,1]', (pos_i + [-1,-1])*[n,1]', (pos_i + [-2, 0])*[n,1]'}
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
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[n,1]'
                            case (pos_i + [0, -1])*[n,1]'
                                P_temp = (1 - P_WIND);
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2)); 
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + (P_temp - P(i, j, action));
                            case pos_i*[n,1]'
                                P_temp = P_WIND * 0.25;
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case {(pos_i + [-1, -1])*[n,1]', (pos_i + [1,-1])*[n,1]', (pos_i + [0, -2])*[n,1]'}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                    P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
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
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2 ] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[n,1]'
                            case (pos_i + [0, 1])*[n,1]'
                                P_temp = (1 - P_WIND);
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2)); 
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case pos_i*[n,1]'
                                P_temp = P_WIND * 0.25;
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case {(pos_i + [-1, 1])*[n,1]', (pos_i + [1,1])*[n,1]', (pos_i + [0, 2])*[n,1]'}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
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
                    %for j = [(i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[n,1]'
                            case (pos_i + [1, 0])*[n,1]'
                                P_temp = (1 - P_WIND);
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case pos_i*[n,1]'
                                P_temp = P_WIND * 0.25;
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case {(pos_i + [1, 1])*[n,1]', (pos_i + [1,-1])*[n,1]', (pos_i + [2, 0])*[n,1]'}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                    P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
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
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2 ] % possible reachable state indexs
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[n,1]'
                            case (pos_i + [0, 0])*[n,1]'
                                P_temp = (1 - P_WIND);
                                P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                            case {(pos_i + [0, 1])*[n,1]', (pos_i + [0,-1])*[n,1]', (pos_i + [1, 0])*[n,1]', (pos_i + [-1, 0])*[n,1]'}
                                if binarySearch(treeList,n*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                    P_IMNORMAL_TO_BASE(i, action) = P_IMNORMAL_TO_BASE(i, action) + P_temp - P(i,j,action);
                                else
                                    P_IMNORMAL_TO_BASE(i,action) = P_IMNORMAL_TO_BASE(i,action) + P_WIND * 0.25;
                                end
                                
                        end
                    end
                end                
        end
    end

    % 
    
end

% UPDATE THE PROB TO BASE
for i = 1:2:K
    for action = 1:5
        P(i,BASE_STATE_INDEX,action) = P(i, BASE_STATE_INDEX,action) + P_IMNORMAL_TO_BASE(i, action);
    end
end

% COPY TO THE EVEN INDEX SUB TRANSITION MATRIX
% TO DO: 1. copy the odd index pair to the even index pair (i,j,u) 
% 2. consider following cases: 
% from FREE WITHOUT PACKAGE to PICKUP;
% from FREE WITH PACKAGE to BASE (lose one's package);
% from FREE WITH PACKAGE to DROPOFF
% IT IS IMPOSSIBLE THAT ONE IS IN PICKUP WITHOUT A PACKAGE. 
% Or just add P(i+1,j+1) after each line above in the odd case.
for i = 2:2:K
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        P(i,:,action) = P(i-1,:,action);
    end
end
% FROM FREE WITHOUT PACKAGE to PICKUP  PICK_UP_INDEX: even
%for i = [Reachable(PICK_UP_INDEX) ]
P(1:2:K, PICK_UP_INDEX, :) = P(1:2:K, PICK_UP_INDEX-1,:);
P(1:2:K, PICK_UP_INDEX-1, :) = 0;
%end
% FROM FREE WITH PACKAGE to BASE: can be from anywhere with even index,
% just the probability of being crashed.
for i = 2:2:K
    idx = BASE_STATE_INDEX;
    P(i, idx, :) = P_IMNORMAL_TO_BASE(i-1, :);
end
% FROM FREE WITH PACKAGE TO DROPOFF: DON'T HAVE TO CHANGE!!!
% FROM TERMINAL_STATE_INDEX: STAY THERE!
P(TERMINAL_STATE_INDEX, :, :) = 0;
P(TERMINAL_STATE_INDEX, TERMINAL_STATE_INDEX, :) = 1;
end

function [treeList, shooterList] = findTree(map)
% return a treeList matrix (nb_tree * 1)
treeList = [ ];
shooterList = [ ];
global TREE SHOOTER n 
for ii = 1:size(map,1)
    for jj = 1:size(map,2)
        if map(ii,jj) == TREE
            treeList = [treeList;
                        ii*n+jj];
        end
        if map(ii,jj) == SHOOTER
            shooterList = [shooterList;
                           ii,jj];
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

function Prob_not_shooted = Survive(shooterlist, stateSpace)
% return the probability matrix (m*n), where each meaningful cell represents the probability
% of surviving the angry resident. We also give value 1 to the tree cells, but
% will never use them in the experiment.
global GAMMA m n K R
Prob_not_shooted = ones(m,n);
nb_shooter = size(shooterlist,1);

for jj = 1:2:K
    pos = stateSpace(jj,1:2);
    for ii = 1:nb_shooter
        shooter = shooterlist(ii,:);
        d = abs(pos(1) - shooter(1)) + abs(pos(2) - shooter(2));
        if d <= R
            Prob_not_shooted(pos(1), pos(2)) = Prob_not_shooted(pos(1), pos(2)) * (1 - (GAMMA/(1 + d)));
        end
    end
end
end
