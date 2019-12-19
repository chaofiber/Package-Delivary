function [ J_opt, u_opt_ind] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER NORTH SOUTH EAST WEST
global TERMINAL_STATE_INDEX

%% Initialization
% intial policy: ALL HOVER, this is a proper policy.
L = 5;
u_opt_ind = HOVER * ones(K-1,1);  
J_opt = zeros(K-1,1);
J_opt_old = -1*ones(K-1,1);
u_old = u_opt_ind;
I = eye(K-1);
P_pol = zeros(K-1, K-1);
G_pol = zeros(K-1, 1);
Q = zeros(L,1);
ite = 0;

%% Handle terminal state
P(TERMINAL_STATE_INDEX,:,:) = [ ];
P(:, TERMINAL_STATE_INDEX,:) = [ ];
G(TERMINAL_STATE_INDEX,:) = [ ];

%% Policy Iteration

while 1
    for i=1:K-1
        P_pol(i,:) = P(i,:,u_old(i));
        G_pol(i) = G(i,u_old(i));
    end
    J_opt = (I - P_pol)\ G_pol;
    for i = 1:K-1
        for action = [HOVER, EAST, NORTH, WEST, SOUTH]
            Q(action) = G(i, action) + P(i, :, action)*J_opt;
        end
        [~, u_opt_ind(i)] = min(Q);
    end
    
    % Theoretically, the termination condition is that the policy stops
    % changing, however, in practive, due to numerical round error, there
    % is chance that the policy finally swithches between two options and
    % keeps oscillating, thus never terminates. Therefore, we add another
    % termination condition, if the optimal cost converges, then terminate
    % the iteration as well.
    if isequal(u_opt_ind,u_old) || max(abs(J_opt-J_opt_old)) < 1e-10
        break;
    end
    u_old = u_opt_ind;
    J_opt_old = J_opt;
    ite = ite + 1;
end
disp("number of policy iteration");
disp(ite);

% Add back the policy and cost-to-go of terminal state
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1,:);0;J_opt(TERMINAL_STATE_INDEX:end,:)];
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1);HOVER;u_opt_ind(TERMINAL_STATE_INDEX:end)];



end


