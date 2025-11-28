function mainCon()
global PROJECT_CONFIG arrivalRate speedInConflictZone preparationDistance...
    totalTime mcIterations
params.numArms = 4;              
params.numLanes = 2;             
params.numTurns = 3;             
params.numStreams = 24;          
params.numConflicts = 108;       
params.vehicleSpeed = speedInConflictZone;    
params.detectionDist = preparationDistance;    
params.mcIterations = mcIterations;     
params.windowSize = 40;      
params.overlapRatio = 0.2;     
params.simulationTime = totalTime;   
params.arrivalRate = arrivalRate;     
bartime = generateConflictTimeMatrix(params);
minInterval = generateMinIntervals(params);
vehicleData = generateVehicleArrivals(params);
fprintf('  - number of vehicles: %d \n', vehicleData.numVehicles);
global model_1
if model_1==1
    bar=3*2*(vehicleData.arm-1) + 2*(vehicleData.r-1) + vehicleData.ls;
    [~,te]=countT(bar,vehicleData.ts);
    delay=(sum(te'-vehicleData.ts))/length(bar);
    fprintf('average delay = %.4f s\n', delay);
    fangcha=0;
    for i=1:length(te)
        fangcha=fangcha+(te(i)-vehicleData.ts(i)-delay)^2;
    end
    bian_yi_xi_shu=((fangcha/length(te))^0.5)/(delay);
    maxdelay=max(te'-vehicleData.ts);
    fprintf('comput time = %.8f \n', toc(tic));
    fprintf('yita = %.2f \n', bian_yi_xi_shu);
    fprintf('max delay = %.2f \n', maxdelay);
else

[optimalSchedule, statistics] = slidingWindowOptimization(...
    vehicleData, bartime, params);
vehicleData = rmfield(vehicleData, 'numVehicles');

fieldNames = fieldnames(vehicleData);
numElements = length(optimalSchedule.vehicleIDs); 

totalVehicles = size(vehicleData.(fieldNames{1}), 1);
vehicles(totalVehicles, 1) = struct();


for i = 1:numElements

    originalIdx = optimalSchedule.vehicleIDs(i);

    for j = 1:length(fieldNames)
        fieldName = fieldNames{j};
        vehicles(originalIdx).(fieldName) = vehicleData.(fieldName)(originalIdx);
    end

    vehicles(originalIdx).te = optimalSchedule.entryTimes(i);
    vehicles(originalIdx).le = optimalSchedule.selectedLanes(i);
    vehicles(originalIdx).barOrder = optimalSchedule.streamIDs(i);
end

vehicles = arrayfun(@(x) setfield(x, 'te2', x.te), vehicles);
vehicles = arrayfun(@(x) setfield(x, 'te', x.te + preparationDistance/(1.7*speedInConflictZone)), vehicles);

save('Vehicles.mat','vehicles');
save('statisticsOfConflict.mat','statistics');
fprintf('  - number of vehicles: %d \n', statistics.totalVehicles);
fprintf('  - average delay: %.2f s\n', statistics.avgDelay);
fprintf('  - max delay: %.2f s\n', statistics.maxDelay);
fprintf('  - total delay: %.2f s\n', statistics.totalDelay);
fprintf('  - yita: %.2f \n', statistics.bian_yi_xi_shu);
fprintf('  - comput time: %.2f s\n', statistics.computationTime);
fprintf('========================================\n');

end