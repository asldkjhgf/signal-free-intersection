function [optimalSchedule, statistics] = slidingWindowOptimization(...
    vehicleData, bartime, params)
startTime = tic; 
optimalSchedule.vehicleIDs = [];    
optimalSchedule.entryTimes = [];     
optimalSchedule.selectedLanes = [];  
optimalSchedule.delays = [];        
optimalSchedule.streamIDs = [];    
optimalSchedule.ts = [];            

statistics.totalVehicles = vehicleData.numVehicles;
statistics.windowsProcessed = 0;
statistics.avgDelay = 0;
statistics.maxDelay = 0;
statistics.totalDelay = 0;

totalVehicles = vehicleData.numVehicles;
ts = vehicleData.ts;
ls = vehicleData.ls;
arm = vehicleData.arm;
r = vehicleData.r;

globalConflictState = zeros(1, params.numConflicts);

processedVehicles = 0;               
carryoverInfo = struct(...       
    'indices', [], ...         
    'entryTimes', [], ...        
    'lanes', [], ...                 
    'ts', [], ...                 
    'arm', [], ...                 
    'r', []);                     
windowCount = 0;

while processedVehicles < totalVehicles
    
    windowCount = windowCount + 1;
    numCarryover = length(carryoverInfo.indices);
    numNewVehicles = min(params.windowSize - numCarryover, ...
                         totalVehicles - processedVehicles);
    newVehicleStart = processedVehicles + 1;
    newVehicleEnd = processedVehicles + numNewVehicles;
    newVehicleIndices = (newVehicleStart:newVehicleEnd)';
    currentVehicleIndices = [carryoverInfo.indices; newVehicleIndices];
    currentWindowSize = length(currentVehicleIndices);
    if numCarryover > 0
        window_ts = [carryoverInfo.ts; ts(newVehicleIndices)];
        window_ls = [carryoverInfo.lanes; ls(newVehicleIndices)]; 
        window_arm = [carryoverInfo.arm; arm(newVehicleIndices)];
        window_r = [carryoverInfo.r; r(newVehicleIndices)];
        
        isCarryover = [true(numCarryover, 1); false(numNewVehicles, 1)];
        carryoverEntryTimes = carryoverInfo.entryTimes;
    else

        window_ts = ts(newVehicleIndices);
        window_ls = ls(newVehicleIndices);
        window_arm = arm(newVehicleIndices);
        window_r = r(newVehicleIndices);
        
        isCarryover = false(currentWindowSize, 1);
        carryoverEntryTimes = [];
    end
    
    tic
    [bestOrder, bestLanes, bestDelay, bestEntryTimes, bestConflictState] = ...
        monteCarloOptimization(window_ts, window_ls, window_arm, window_r, ...
                               bartime, params, globalConflictState);
    fprintf('comput time of this window=%.8f\n',toc);
    
    [sortedEntryTimes, sortIdx] = sort(bestEntryTimes);
    
    sortedIndices = currentVehicleIndices(sortIdx);
    sortedLanes = bestLanes(sortIdx);
    sortedTs = window_ts(sortIdx);
    sortedArm = window_arm(sortIdx);
    sortedR = window_r(sortIdx);
    
    sortedStreamIDs = 3*2*(sortedArm-1) + 2*(sortedR-1) + sortedLanes;
    
    if processedVehicles + numNewVehicles < totalVehicles
        numToCarryover = floor(currentWindowSize * params.overlapRatio);
        numToFinalize = currentWindowSize - numToCarryover;
        
        finalizeIndices = 1:numToFinalize;
        carryoverIndices = (numToFinalize + 1):currentWindowSize;
        
    else
        finalizeIndices = 1:currentWindowSize;
        carryoverIndices = [];
        
    end
    
    finalVehicleIDs = sortedIndices(finalizeIndices);
    finalEntryTimes = sortedEntryTimes(finalizeIndices);
    finalLanes = sortedLanes(finalizeIndices);
    finalTs = sortedTs(finalizeIndices);
    finalStreamIDs = sortedStreamIDs(finalizeIndices);
    
    optimalSchedule.vehicleIDs = [optimalSchedule.vehicleIDs; finalVehicleIDs];
    optimalSchedule.entryTimes = [optimalSchedule.entryTimes; finalEntryTimes];
    optimalSchedule.selectedLanes = [optimalSchedule.selectedLanes; finalLanes];
    optimalSchedule.streamIDs = [optimalSchedule.streamIDs; finalStreamIDs];
    optimalSchedule.ts = [optimalSchedule.ts; finalTs];
    
    
    if ~isempty(finalizeIndices)
        [globalConflictState, adjustedEntryTimes] = countT(finalStreamIDs', finalTs');

        vehicleDelays = adjustedEntryTimes' - finalTs;

        currentOutputSize = length(optimalSchedule.vehicleIDs);
        startIdx = currentOutputSize - length(finalizeIndices) + 1;
        endIdx = currentOutputSize;

        optimalSchedule.entryTimes(startIdx:endIdx) = adjustedEntryTimes';
        optimalSchedule.delays(startIdx:endIdx) = vehicleDelays;

    end
    
    if ~isempty(carryoverIndices)
        carryoverInfo.indices = sortedIndices(carryoverIndices);
        carryoverInfo.entryTimes = sortedEntryTimes(carryoverIndices);
        carryoverInfo.lanes = sortedLanes(carryoverIndices);
        carryoverInfo.ts = sortedTs(carryoverIndices);
        carryoverInfo.arm = sortedArm(carryoverIndices);
        carryoverInfo.r = sortedR(carryoverIndices);
        
    else
        carryoverInfo.indices = [];
        carryoverInfo.entryTimes = [];
        carryoverInfo.lanes = [];
        carryoverInfo.ts = [];
        carryoverInfo.arm = [];
        carryoverInfo.r = [];
    end
    
    processedVehicles = processedVehicles + numNewVehicles;
end

a=[optimalSchedule.vehicleIDs,optimalSchedule.entryTimes,optimalSchedule.selectedLanes,optimalSchedule.delays',optimalSchedule.streamIDs,optimalSchedule.ts];
[~,entryTimes]=countT(a(:,5),a(:,6));
optimalSchedule.entryTimes=entryTimes';
optimalSchedule.delays=optimalSchedule.entryTimes-optimalSchedule.ts;
fangcha=0;
for i=1:length(optimalSchedule.entryTimes)
    fangcha=fangcha+(optimalSchedule.entryTimes(i)-optimalSchedule.ts(i)-sum(optimalSchedule.delays)/length(optimalSchedule.entryTimes))^2;
end
statistics.bian_yi_xi_shu=((fangcha/length(optimalSchedule.entryTimes))^0.5)/(sum(optimalSchedule.delays)/length(optimalSchedule.entryTimes));
statistics.windowsProcessed = windowCount;
statistics.avgDelay = mean(optimalSchedule.delays);
statistics.maxDelay = max(optimalSchedule.delays);
statistics.totalDelay = sum(optimalSchedule.delays);
statistics.computationTime = toc(startTime);
end