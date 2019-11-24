# Package-Delivary
Dynamic Programming and Optimal Control project, reference source: https://idsc.ethz.ch/education/lectures/optimal-control.html

# Task Pipeline
## Compute Index of the Terminal State ```t``` (easy)
## Compute Probability Transition Matrix P(K,K,L)

```
P(t,t,u) = 1 for all u, P(t,v,u) = 0, for all u!=t
```
Define ```NormalReachable{i}``` Set for each state i, which represents all possible state can be reached from i, Also define ```IMNormalReachable{i}```, which is ```{base}``` and is the resulting state from crash event (out of border, hitting a tree or shooting by an angry resident)
### Key ideas
Construct a tree list containing positions of trees T; Sort these trees in order;
Given index i, we can obtain its position immediately, consider all 13 possible next states instead of all states. 
Computer the number of trees n between the given place and the most left and most right possible positions;
Use n as the bound for searching all other rest index among 13 possible next states;
Memorize ```NormalReachable{i}``` and ```IMNormalReachable{i}```;
```
Search range: {start,[i-2n-2,i-2n+2+2p1],[i-4,i+4],[i+2n-2p2,i+2n+2],end}
## Compute Cost Function Matrix
```
G(i,u) = 1 + (\sum_{j\in NormalReachable{i}} P(i,j,u))*10
```

