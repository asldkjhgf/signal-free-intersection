function bartime = generateConflictTimeMatrix(params)
global Bars
bartime = NaN(params.numStreams, params.numConflicts);
bartime = Bars;
end
