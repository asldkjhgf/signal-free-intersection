clc;clear;
fprintf('==================================\n');
fprintf('start\n');
fprintf('==================================\n');
turn{4}=[0.33 0.33 0.33]; turn{1}=[0.5 0.25 0.25]; turn{2}=[0.25 0.5 0.25]; turn{3}=[0.25 0.25 0.5];
global totalTime arrivalRate turnProbs mcIterations
totalTime=100;
k=1;
for volume=1000:1000:6000
   for i=4
        fprintf('volume:%.f,turn:%.f\n',volume,i);
        turnProbs = turn{i};
        mcIterations=100;
        arrivalRate=volume/3600;
        mainInp;
        mainCon();
   end
end

