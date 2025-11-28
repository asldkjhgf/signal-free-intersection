function [bestOrder, bestLanes, bestDelay, bestEntryTimes, bestConflictState] = ...
    mctsOptimization(window_ts, window_ls, window_arm, window_r, ...
                     bartime, params, ...
                     initialConflictState)

n = length(window_ts);
numIterations = params.mcIterations;
if isfield(params, 'explorationParam')
    C = params.explorationParam;
else
    C = 1.414;  
end
armGroups = cell(4, 1);
armPointers = ones(4, 1); 
for i = 1:4
    armGroups{i} = find(window_arm == i);
end
root = struct();
root.order = [];          
root.lanes = [];         
root.barOrder = [];       
root.reorderedTs = [];    
root.armPointers = ones(4, 1);  
root.visits = 0;          
root.totalReward = 0;     
root.children = [];       
root.parent = [];         
root.isFullyExpanded = false;
bestDelay = inf;
bestOrder = [];
bestLanes = [];
bestEntryTimes = [];
bestConflictState = initialConflictState;

for iter = 1:numIterations
    
    %% Selection
    node = root;
    while ~isempty(node.children) && node.isFullyExpanded
        node = selectBestChild(node, C);
    end
    
    %% Expansion
    if length(node.order) < n && ~node.isFullyExpanded
        node = expandNode(node, armGroups, window_ts, window_arm, window_r,window_ls);
    end
    
    %% Simulation
    [finalOrder, finalLanes, avgDelay] = simulateRandomCompletion(...
        node, armGroups, window_ts, window_arm, window_r, n,window_ls);
    
    %% Backpropagation
    reward = -avgDelay; 
    backpropagate(node, reward);
    
    if avgDelay < bestDelay
        bestDelay = avgDelay;
        bestOrder = finalOrder;
        bestLanes = finalLanes;
        
        [bestEntryTimes, bestConflictState] = calculateEntryTimes(...
            finalOrder, finalLanes, window_ts, window_arm, window_r, n);
    end
end
bestOrder = bestOrder(:);
bestLanes = bestLanes(:);
bestEntryTimes = bestEntryTimes(:);

end

function bestChild = selectBestChild(node, C)
    
    numChildren = length(node.children);
    ucbValues = zeros(numChildren, 1);
    
    for i = 1:numChildren
        child = node.children{i};
        if child.visits == 0
            ucbValues(i) = inf;  
        else
            avgReward = child.totalReward / child.visits;
            exploration = C * sqrt(log(node.visits) / child.visits);
            ucbValues(i) = avgReward + exploration;
        end
    end
    
    [~, bestIdx] = max(ucbValues);
    bestChild = node.children{bestIdx};
end

function newNode = expandNode(node, armGroups, ts, arm, r,ls)    
    availableVehicles = [];
    availableArms = [];
    
    for armIdx = 1:4
        if node.armPointers(armIdx) <= length(armGroups{armIdx})
            vehIdx = armGroups{armIdx}(node.armPointers(armIdx));
            availableVehicles = [availableVehicles, vehIdx];
            availableArms = [availableArms, armIdx];
        end
    end
    
    if isempty(availableVehicles)
        node.isFullyExpanded = true;
        newNode = node;
        return;
    end
    
    if isempty(node.children)
        node.children = cell(0);
    end
    
    if length(availableVehicles) > length(node.children)
        unexpandedIdx = length(node.children) + 1;
        if unexpandedIdx <= length(availableVehicles)
            selectedVehIdx = availableVehicles(unexpandedIdx);
            selectedArm = availableArms(unexpandedIdx);
        else
            node.isFullyExpanded = true;
            newNode = node;
            return;
        end
    else
        node.isFullyExpanded = true;
        newNode = node;
        return;
    end
    
        
    [selectedLane, selectedBar, selectedEntryTime] = chooseBestLane(...
        selectedVehIdx, node.barOrder, node.reorderedTs, ts, arm, r,ls);
    
    newNode = struct();
    newNode.order = [node.order, selectedVehIdx];
    newNode.lanes = [node.lanes, selectedLane];
    newNode.barOrder = [node.barOrder, selectedBar];
    newNode.reorderedTs = [node.reorderedTs, ts(selectedVehIdx)];
    newNode.armPointers = node.armPointers;
    newNode.armPointers(selectedArm) = newNode.armPointers(selectedArm) + 1;
    newNode.visits = 0;
    newNode.totalReward = 0;
    newNode.children = [];
    newNode.parent = node;
    newNode.isFullyExpanded = false;
    
    node.children{end+1} = newNode;
end

function [bestLane, bestBar, bestEntryTime] = chooseBestLane(...
    vehIdx, currentBarOrder, currentTs, ts, arm, r,ls)

    global model_4
    if model_4==1
        bestLane=ls(vehIdx);
        bestBar = 3*2*(arm(vehIdx)-1) + 2*(r(vehIdx)-1) + ls(vehIdx);
        testBarOrder=[currentBarOrder, bestBar];
        testTs=[currentTs, ts(vehIdx)];
        [~, te] = countT(testBarOrder, testTs);
        bestEntryTime=te(end);
    else
    b1 = 3*2*(arm(vehIdx)-1) + 2*(r(vehIdx)-1) + 1;  
    b2 = 3*2*(arm(vehIdx)-1) + 2*(r(vehIdx)-1) + 2; 
    
    testBarOrder1 = [currentBarOrder, b1];
    testTs1 = [currentTs, ts(vehIdx)];
    [~, te1] = countT(testBarOrder1, testTs1);
    delay1 = te1(end) - ts(vehIdx);
    
    testBarOrder2 = [currentBarOrder, b2];
    testTs2 = [currentTs, ts(vehIdx)];
    [~, te2] = countT(testBarOrder2, testTs2);
    delay2 = te2(end) - ts(vehIdx);
    
    if delay1 <= delay2
        bestLane = 1;
        bestBar = b1;
        bestEntryTime = te1(end);
    else
        bestLane = 2;
        bestBar = b2;
        bestEntryTime = te2(end);
    end
    end
end

function [finalOrder, finalLanes, avgDelay] = simulateRandomCompletion(...
    node, armGroups, ts, arm, r, totalVehicles,ls)    
    currentOrder = node.order;
    currentLanes = node.lanes;
    currentBarOrder = node.barOrder;
    currentTs = node.reorderedTs;
    currentArmPointers = node.armPointers;
    
    while length(currentOrder) < totalVehicles
        availableVehicles = [];
        availableArms = [];
        
        for armIdx = 1:4
            if currentArmPointers(armIdx) <= length(armGroups{armIdx})
                vehIdx = armGroups{armIdx}(currentArmPointers(armIdx));
                availableVehicles = [availableVehicles, vehIdx];
                availableArms = [availableArms, armIdx];
            end
        end
        
        if isempty(availableVehicles)
            break;
        end
        
        [~, randIdx] = min(ts(availableVehicles));
        selectedVehIdx = availableVehicles(randIdx);
        selectedArm = availableArms(randIdx);
        
        [selectedLane, selectedBar, ~] = chooseBestLane(...
            selectedVehIdx, currentBarOrder, currentTs, ts, arm, r,ls);
        
        currentOrder = [currentOrder, selectedVehIdx];
        currentLanes = [currentLanes, selectedLane];
        currentBarOrder = [currentBarOrder, selectedBar];
        currentTs = [currentTs, ts(selectedVehIdx)];
        currentArmPointers(selectedArm) = currentArmPointers(selectedArm) + 1;
    end
    
    [~, te] = countT(currentBarOrder, currentTs);
    totalDelay = sum(te - currentTs);
    avgDelay = totalDelay / length(currentOrder);
    
    finalOrder = zeros(totalVehicles, 1);
    finalLanes = zeros(totalVehicles, 1);
    for i = 1:length(currentOrder)
        finalOrder(i) = currentOrder(i);
        finalLanes(currentOrder(i)) = currentLanes(i);
    end
end
function backpropagate(node, reward)
    current = node;
    while ~isempty(current)
        current.visits = current.visits + 1;
        current.totalReward = current.totalReward + reward;
        if isempty(current.parent)
            break;
        end
        current = current.parent;
    end
end

function [entryTimes, conflictState,aveDelay] = calculateEntryTimes(...
    order, lanes, ts, arm, r, n)
    
    entryTimes = zeros(n, 1);
    barOrder = zeros(1, n);
    reorderedTs = zeros(1, n);
    
    for i = 1:n
        vehIdx = order(i);
        le = lanes(vehIdx);
        barOrder(i) = 3*2*(arm(vehIdx)-1) + 2*(r(vehIdx)-1) + le;
        reorderedTs(i) = ts(vehIdx);
    end
    
    [conflictState, te] = countT(barOrder, reorderedTs);
    aveDelay = sum(te - reorderedTs);

    for i = 1:n
        vehIdx = order(i);
        entryTimes(vehIdx) = te(i);
    end
end