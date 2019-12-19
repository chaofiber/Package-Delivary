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
[m, n] = size(map);
L = 5;

shooterList = findShooter(map);
Prob_Survive = Survive(shooterList, stateSpace);

% DEFINE THE MAP_TO_INDEX MATRIX
idxList = MaptoIndex(stateSpace);

% DIFINE WIND DIRECTIONS
Direction = zeros(5,2);
Direction(EAST,:) = [1, 0];
Direction(WEST,:) = [-1,0];
Direction(NORTH,:) = [0,1];
Direction(SOUTH,:) = [0,-1];
Direction(HOVER,:) = [0,0];

P_IMNORMAL_TO_BASE = -1*ones(K,L); % IF P_IMNORMAL_TO_BASE(i,u) = -1, then such (i,u) pair is not allowed.
G = Inf(K,L);
%% COMPUTE P_IMNORMAL_TO_BASE MATRIX
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
                            p_normal = p_normal + P_temp*Prob_Survive(pos_j(1), pos_j(2));
                        end
                    end
                end
                % LUCKY! NO WIND!!! STAY WHERE YOU ARE!
                P_temp = 1 - P_WIND;
                p_normal = p_normal + P_temp * Prob_Survive(ppos(1), ppos(2));
                % OTHERWISE, GO BACK TO BASE! -.- CRASH!
                P_IMNORMAL_TO_BASE(i, action) = 1 - p_normal;
            end
            
        end
    end

    
end

%% COMPUTE STAGE COST FUNCTION: 
% G(i,u) = 1 + P_IMNORMAL_TO_BASE(i,u) * (Nc-1);
% SPECIAL CASE:
% 1. G(TERMINAL_STATE_INDEX,:) = 0 % FROM TERMINATION, YOU ARE DONE!!!
% 2. G(i,NOT ALLOABLW ACTION) = Inf
for i = 1:2:K
    for action = [WEST, SOUTH, NORTH, EAST, HOVER]
        if P_IMNORMAL_TO_BASE(i,action) ~= -1 % This action is allowed
            G(i,action) = 1 + P_IMNORMAL_TO_BASE(i,action) * (Nc - 1);
        end
    end
end

% UPDATE EVEN CASES AND TERMINAL_STATE
G(2:2:K,:) = G(1:2:K,:);
G(TERMINAL_STATE_INDEX,:) = 0;
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

