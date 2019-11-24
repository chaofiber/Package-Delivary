# Package-Delivary
Dynamic Programming and Optimal Control project, reference source: https://idsc.ethz.ch/education/lectures/optimal-control.html

# Task Pipeline
## Compute index of the terminal state t (easy)
## COmpute probability transition matrix P(i,j,u)
	###
	1. `P(t,t,u) = 1 for all u, P(t,v,u) = 0, for all u!=t`
	2. Define $NormalReachable{i}$ Set for each state $i$, which represents all possible state can be reached from $i$, Also define $IMNormalReachable{i}$, which is {base} and is the resulting state from crash event.
	
