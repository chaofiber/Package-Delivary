p = [];
global K stateSpace map
treeList = findTree(map);
[m,n] = size(map);

for i = 1:2:K
    % Possible position can be reached from state i stateSpace(i) = (i,0/1)
    %                        I
    %                      F G H
    %                    A B i D E 
    %                      J K L
    %                        M
    
    pos_state = stateSpace(i,:,:);
    posA = [pos_state(1)-2, pos_state(2)];
    %posE = [posI_x, pos_y+2];
    idxI = n*pos_state(1) + pos_state(2);
    idxA = n*posA(1) + posA(2);
    p = [p;binarySearch(treeList, idxI, false) - binarySearch(treeList, idxA, false),idxI,idxA];
end

function treeList = findTree(map)
% return an array treeList (number of tree * 1)
global TREE
treeList = [ ];
n = size(map,2);
for i = 1:size(map,1)
    for j = 1:size(map,2)
        if map(i,j) == TREE
            treeList = [treeList;
                        i*n+j];
        end

    end
end
end