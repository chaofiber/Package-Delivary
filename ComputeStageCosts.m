function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX
    
%% INITILIZATION
% find the treeList (number of tree * 1)
global m n
[m, n] = size(map);
nn = 2*n; % To AVOID SINGULAR CASES!!!
L = 5;
[treeList, shooterList] = findTree(map);
Prob_Survive = Survive(shooterList, stateSpace);

P_IMNORMAL_TO_BASE = -1*ones(K,L); % IF P_IMNORMAL_TO_BASE(i,u) = -1, then such (i,u) pair is not allowed.
G = Inf(K,L);
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
    idxI = nn*pos_i(1) + pos_i(2);
    idxA = nn*pos_A(1) + pos_A(2);
    idxE = nn*pos_E(1) + pos_E(2);
    p = binarySearch(treeList, idxI, false) - binarySearch(treeList, idxA, false);
    q = binarySearch(treeList, idxE, false) - binarySearch(treeList, idxI, false);
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        switch action
            case WEST
                % Check if this action is allowed ( NOT HITTING A TREE AND
                % NOT OUT OF BORDER !!!)
                % ppos: the cell the drone is supposed to be in if no wind
                % happens
                ppos = pos_i + [-1, 0];
                if binarySearch(treeList,nn*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    p_normal = 0;
                    % Search possible states after WEST action and the wind
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i ] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[nn,1]'
                            case (pos_i + [-1, 0])*[nn,1]'
                                P_temp = (1 - P_WIND); 
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case pos_i*[nn,1]'
                                P_temp = P_WIND * 0.25;
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case {(pos_i + [-1, 1])*[nn,1]', (pos_i + [-1,-1])*[nn,1]', (pos_i + [-2, 0])*[nn,1]'}
                                if binarySearch(treeList,nn*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                end
                                
                        end
                    end
                    P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
                end
                
            case SOUTH
                ppos = pos_i + [0, -1];
                if binarySearch(treeList,nn*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    p_normal = 0;
                    % Search possible states after WEST action and the wind
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[nn,1]'
                            case (pos_i + [0, -1])*[nn,1]'
                                P_temp = (1 - P_WIND);
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case pos_i*[nn,1]'
                                P_temp = P_WIND * 0.25;
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case {(pos_i + [-1, -1])*[nn,1]', (pos_i + [1,-1])*[nn,1]', (pos_i + [0, -2])*[nn,1]'}
                                if binarySearch(treeList,nn*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                end
                                
                        end
                    end
                    P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
                end    
                
            case NORTH
                ppos = pos_i + [0, 1];
                if binarySearch(treeList,nn*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    p_normal = 0;
                    % Search possible states after WEST action and the wind
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2 ] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end  % ALSO EXCLUDE THE CASE WHERE J IS OUTOFBORDER!!!
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[nn,1]'
                            case (pos_i + [0, 1])*[nn,1]'
                                P_temp = (1 - P_WIND);
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case pos_i*[nn,1]'
                                P_temp = P_WIND * 0.25;
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case {(pos_i + [-1, 1])*[nn,1]', (pos_i + [1,1])*[nn,1]', (pos_i + [0, 2])*[nn,1]'}
                                if binarySearch(treeList,nn*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                end
                                
                        end
                    end
                    P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
                end
                
            case EAST
                ppos = pos_i + [1, 0];
                if binarySearch(treeList,nn*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    p_normal = 0;
                    % Search possible states after WEST action and the wind
                    %for j = [(i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2] % possible reachable state indexs]
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[nn,1]'
                            case (pos_i + [1, 0])*[nn,1]'
                                P_temp = (1 - P_WIND);
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case pos_i*[nn,1]'
                                P_temp = P_WIND * 0.25;
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case {(pos_i + [1, 1])*[nn,1]', (pos_i + [1,-1])*[nn,1]', (pos_i + [2, 0])*[nn,1]'}
                                if binarySearch(treeList,nn*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                end
                                
                        end
                    end
                    P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
                end
                
            case HOVER
                ppos = pos_i + [0, 0];
                if binarySearch(treeList,nn*ppos(1)+ppos(2),true) ~= -1 && ~OutOfBorder(m,n,ppos)
                    p_normal = 0;
                    % Search possible states after WEST action and the wind
                    %for j = [ (i-2*n):2:(i-2*n+2+2*p),i-4:2:i+4, i+2*n-2*q:2:i+2*n+2 ] % possible reachable state indexs
                    for j = [ (i-4*n:2:i+4*n)]
                        if j<1 || j>K; continue; end
                    	pos_state_j = stateSpace(j,:,:);
                        pos_j = pos_state_j(1:2);
                        switch pos_j*[nn,1]'
                            case (pos_i + [0, 0])*[nn,1]'
                                P_temp = (1 - P_WIND);
                                p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            case {(pos_i + [0, 1])*[nn,1]', (pos_i + [0,-1])*[nn,1]', (pos_i + [1, 0])*[nn,1]', (pos_i + [-1, 0])*[nn,1]'}
                                if binarySearch(treeList,nn*pos_j(1)+pos_j(2),true) ~= -1 && ~OutOfBorder(m,n,pos_j)
                                    P_temp = P_WIND * 0.25;
                                    p_normal = p_normal + P_temp * Prob_Survive(pos_j(1), pos_j(2));
                                end
                                
                        end
                    end
                    P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
                end                
        end
    end

    
end

%% COMPUTE STAGE COST FUNCTION: 
% G(i,u) = 1 + P_IMNORMAL_TO_BASE(i,u)*10;
% SPECIAL CASE:
% 1. G(TERMINAL_STATE_INDEX,:) = 0 % FROM TERMINATION, YOU ARE DONE!!!
% 2. G(i,NOT ALLOABLW ACTION) = Inf
for i = 1:2:K
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        if P_IMNORMAL_TO_BASE(i,action) ~= -1
            G(i,action) = 1 + P_IMNORMAL_TO_BASE(i,action) * (Nc - 1);
        end
    end
end

% UPDATE EVEN CASES
G(2:2:K,:) = G(1:2:K,:);
G(TERMINAL_STATE_INDEX,:) = 0;
end

function [treeList, shooterList] = findTree(map)
% return a treeList matrix (nb_tree * 1)
treeList = [ ];
shooterList = [ ];
global TREE SHOOTER n 
nn = 2*n;
for ii = 1:size(map,1)
    for jj = 1:size(map,2)
        if map(ii,jj) == TREE
            treeList = [treeList;
                        ii*nn+jj];
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

