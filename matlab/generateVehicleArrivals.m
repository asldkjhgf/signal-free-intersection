function vehicleData = generateVehicleArrivals(params)
lambda = params.arrivalRate;
T = params.simulationTime;
expectedVehicles = lambda * T;
numVehicles = poissrnd(expectedVehicles);
interArrivalTimes = exprnd(1/lambda, numVehicles, 1);
ts = cumsum(interArrivalTimes);
ts = ts(ts <= T);
numVehicles = length(ts);
ls = randi([1, 2], numVehicles, 1);
arm = randi([1, params.numArms], numVehicles, 1);
global turnProbs
turnCumProbs = cumsum(turnProbs);

r = zeros(numVehicles, 1);
for i = 1:numVehicles
    randVal = rand;
    if randVal < turnCumProbs(1)
        r(i) = 1;  % left
    elseif randVal < turnCumProbs(2)
        r(i) = 2;  % straght
    else
        r(i) = 3;  % right
    end
end
[ts, sortIdx] = sort(ts);
ls = ls(sortIdx);
arm = arm(sortIdx);
r = r(sortIdx);
vehicleData.numVehicles = numVehicles;
vehicleData.ts = ts;
vehicleData.ls = ls;
vehicleData.arm = arm;
vehicleData.r = r;
vehicleData.id = 1:numVehicles;
end
