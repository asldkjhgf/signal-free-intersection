function minInterval = generateMinIntervals(params)
minInterval = zeros(params.numConflicts, 1);
divergingInterval=1;mergingInterval=1;crossingInterval=1.5;
b=[1 2 5 6 9 10 13 14];
c=[3 4 7 8 11 12 15 16];
for i=1:params.numConflicts
    if ismember(i,b)
        minInterval(i)=divergingInterval;
    elseif ismember(i,c)
        minInterval(i)=mergingInterval;
    else
        minInterval(i)=crossingInterval;
    end
end

end
