function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
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

global K
global NORTH SOUTH EAST WEST HOVER
global TERMINAL_STATE_INDEX

%% Initialization
% intial policy: HOVER for all states
u_opt_ind = HOVER * ones(K-1,1);  
J_opt = zeros(K-1,1);
%u_opt_ind=[1;1;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;3;2;2;2;2;2;2;2;2;2;3;2;2;1;1;1;1;1;1;1;1;3;1;3;1;3;3;3;3;3;3;3;3;2;2;2;2;2;2;2;2;2;2;2;2;2;2;2;2;1;1;1;1;1;1;1;1;1;1;3;1;3;1;3;1;3;1;3;2;2;2;2;2;2;2;2;2;4;4;4;4;1;1;1;1;1;1;1;1;1;1;1;1;3;1;3;1;3;4;2;2;2;2;2;2;2;4;2;2;4;4;4;4;1;1;1;1;1;4;1;4;1;4;1;1;1;1;3;4;3;4;2;4;4;4;4;4;1;1;1;4;4;4;1;4;1;4;1;1;1;4;3;4;3;4;2;2;1;1;3;3;1;1;3;4;3;4;3;4;1;1;1;1;1;1;3;4;3;4;3;2;1;1;3;3;1;1;1;4;1;2;1;2;1;1;1;1;1;1;3;4;3;4;3;2;3;3;3;3;3;3;2;2;1;1;3;4;4;2;1;1;3;4;3;4;3;2;3;3;3;3;2;2;2;2;1;1;3;4;3;2;1;1;1;4;3;4;3;2;5;3;2;3;2;3;2;3;2;2;2;2;2;2;1;1;3;4;3;2;1;1;1;4;1;4;1;2;1;2;1;2;1;2;4;3;2;2;2;2;2;2;2;2;2;2;2;2;1;1;1;4;1;2;1;2;1;2;1;1;1;1;1;4;1;4;1;4;1;2;1;2;1;2;4;2;4;2;4;4;4;4;2;2;2;2;2;2;1;1;1;4;4;4;4;2;4;2;1;1;1;1;1;4;1;4;1;4;1;4;4;2;4;2;4;2;4;2;4;4;4;4;4;4;2;2;2;2;1;1;1;4;1;4;1;2;1;2;1;1;1;1;1;1;1;4;1;4;4;4;4;4;4;2;4;2;4;2;4;4;4;4;4;4;4;4;2;2;1;4;1;4;1;4;1;4;1;1;1;1;1;1;4;4;4;4;4;4;4;4;4;4;4;2;2;2;2;2;4;4;4;4;4;4];

max_val_iter = 200;
%max_pol_iter = 10000;

G(isinf(G)) = 100000;
row_col_idx = [1:TERMINAL_STATE_INDEX-1,TERMINAL_STATE_INDEX+1:K];

P_plus = P(row_col_idx,row_col_idx,:);
G_plus = G(row_col_idx,:);
I = eye(K-1);

%% Calculate optimal policy

pre_value = J_opt;
pre_policy = u_opt_ind;
while 1
    %new_value = Cal_Value(pre_policy, pre_value, max_val_iter, P, G);
    new_value = (I - P_plus(:,:,pre_policy))\G_plus(:,pre_policy);
    new_policy = Cal_Policy(pre_policy, new_value, P, G);
    
    if isequal(new_policy,pre_policy)
        break;
    end
    pre_policy = new_policy;
    %pre_value = new_value;
end

J_opt = new_value;
u_opt_ind = new_policy;

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

end

function PolicyImprove = Cal_Policy(policy_now, value_now, P, G)
global NORTH SOUTH EAST WEST HOVER K
% Improve the policy by minimizing the value
L = 5;
PolicyImprove = policy_now;
cost_u = zeros(L,1);

for i = 1:1:K
    for action = [NORTH,SOUTH,EAST,WEST,HOVER]
        cost_u(action) = G(i, action) + P(i,:,action) * value_now;
    end
    [~, ind] = min(cost_u);
    PolicyImprove(i)= ind;
end

end

function Value_now = Cal_Value(policy_now, value_pre, max_iter, P, G)
global K
% Calculate the value for current policy by iteration
iter = 0;
Value_now = value_pre;
while 1   
    for i = 1:1:K
        Value_now(i,1) = G(i,policy_now(i))+ P(i,:,policy_now(i))* value_pre;
    end
   
    % Define the condition to end iteration
    if max(abs(Value_now-value_pre)) < 1e-6  || iter > max_iter
        break;
    end
    
    iter = iter + 1;
    value_pre = Value_now;
end

end
