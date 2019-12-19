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

%% INITILIZATION
global m n 
[m, n] = size(map);
L = 5;
% find the position of all shooters and compute a probability matrix of
% surviving, which will be multiplied in the calculation of transition
% probabilities matrix
shooterList = findShooter(map);
Prob_Survive = Survive(shooterList, stateSpace);

% find the state index of every cell in the map
idxList = MaptoIndex(stateSpace);
% define the change of coordinates only by inputs or winds of different direction
Direction = zeros(5,2);
Direction(EAST,:) = [1,0];
Direction(WEST,:) = [-1,0];
Direction(NORTH,:) = [0,1];
Direction(SOUTH,:) = [0,-1];
Direction(HOVER,:) = [0,0];

[x,y] = find(map==BASE);
BASE_STATE_INDEX = find(ismember(stateSpace,[x,y,0],'rows'));

[x,y] = find(map==PICK_UP);
PICK_UP_INDEX = find(ismember(stateSpace,[x,y,1],'rows'));

P = zeros(K,K,L);
% define a matrix that shows the probability of return to the base due to
% the crash of each state
P_IMNORMAL_TO_BASE = zeros(K,L);

%% COMPUTE ODD SUB TRANSITION MATRIX FIRST
for i = 1:2:K
    pos_i = stateSpace(i,1:2);
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        % Check if this action is allowed ( NOT HITTING A TREE AND
        % NOT OUT OF BORDER !!!)
        % ppos: the cell the drone is supposed to be in if no wind happens
        ppos = pos_i + Direction(action,:);
        if ~OutOfBorder(m,n,ppos)
            j_temp = idxList(ppos(1), ppos(2));
            if j_temp ~= -1
                % This action is valid only if it neither hits the
                % tree nor hits the border
                p_normal = 0;
                % WIND COMES!!!!!
                for wind = [WEST, SOUTH, NORTH, EAST]   
                    pos_j = ppos + Direction(wind,:);
                    if ~OutOfBorder(m,n,pos_j)
                        j = idxList(pos_j(1), pos_j(2));
                        if j ~= -1 % NOT A TREE
                            P_temp = P_WIND * 0.25; 
                            P(i, j, action) = P_temp * Prob_Survive(pos_j(1), pos_j(2));
                            % copy the result to the cell with even state index
                            P(i+1, j+1, action) = P(i, j, action);
                            p_normal = p_normal + P(i,j,action);
                        end
                    end
                end
                % LUCKY! NO WIND!!! STAY WHERE YOU ARE!
                P_temp = 1 - P_WIND;
                P(i,j_temp,action) = P_temp * Prob_Survive(ppos(1), ppos(2));
                % copy the result to the cell with even state index
                P(i+1, j_temp+1, action) = P(i, j_temp, action);
                p_normal = p_normal + P(i,j_temp,action);
                % OTHERWISE, GO BACK TO BASE! -.- CRASH!!
                P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
            end
            
        end
    end

    
end

% correct some trivial errors in simple copying
% consider following cases: 
% from FREE WITHOUT PACKAGE to PICKUP;
% from FREE WITH PACKAGE to BASE (crash and lose its package);
% from FREE WITH PACKAGE to DROPOFF
% IT IS IMPOSSIBLE THAT ONE IS IN PICKUP WITHOUT A PACKAGE. 


% FROM FREE WITHOUT PACKAGE to PICKUP  
% correct the probability of transition from other states to pickup state
% since PICK_UP index is even
P(1:2:K, PICK_UP_INDEX, :) = P(1:2:K, PICK_UP_INDEX-1,:);
P(1:2:K, PICK_UP_INDEX-1, :) = 0;

% FROM FREE WITH PACKAGE to BASE: can be from anywhere with even index,
% it is just the probability of being crashed.
for i = 2:2:K
    P(i, BASE_STATE_INDEX, :) = P_IMNORMAL_TO_BASE(i-1, :);
end

% UPDATE THE PROBABILITY TO BASE
for i = 1:2:K
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        P(i,BASE_STATE_INDEX,action) = P(i, BASE_STATE_INDEX,action) + P_IMNORMAL_TO_BASE(i,action);
    end
end

% FROM FREE WITH PACKAGE TO DROPOFF: DON'T HAVE TO CHANGE!!!
% FROM TERMINAL_STATE_INDEX: STAY THERE!
P(TERMINAL_STATE_INDEX, :, :) = 0;
P(TERMINAL_STATE_INDEX, TERMINAL_STATE_INDEX, :) = 1;
end

function shooterList = findShooter(map)
% return a treeList matrix (nb_tree * 1)
shooterList = [ ];
global SHOOTER 
for ii = 1:size(map,1)
    for jj = 1:size(map,2)
        if map(ii,jj) == SHOOTER
            shooterList = [shooterList;
                           ii,jj];
        end

    end
end
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

function indexList = MaptoIndex(stateSpace)
% return a matrix (m * n), indexList(i,j) = stateIndex of position(i,j),
% indexList(i,j)= -1 means there is a tree in this position
global K m n
indexList = -1*ones(m,n);
for i = 1:2:K
    pos = stateSpace(i,1:2);
    indexList(pos(1), pos(2)) = i;
end
end
