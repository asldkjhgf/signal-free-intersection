function [t,te] = countT(barOrder, ts)
global Bars TrackbarsPoints temp_tao nk
t=zeros(1,nk);
if isempty(barOrder)
    te=[];
else
    for i=1:length(barOrder)
        temp = Bars(barOrder(i),:)-t;
        nonZeroIndex=TrackbarsPoints(barOrder(i),TrackbarsPoints(barOrder(i),:)~=0);
        value=temp(nonZeroIndex);
        [~, min_index]= min(value);
        minIndex=nonZeroIndex(min_index);
        kikTime=t(minIndex);
        arrivalTime=max(kikTime-Bars(barOrder(i),minIndex),ts(i));
        [TimeAdded] = addBarTime(Bars(barOrder(i),:),barOrder(i),arrivalTime);
        arrivalBar=TimeAdded+Bars(barOrder(i),:);
        t(nonZeroIndex) = arrivalBar(nonZeroIndex);
        t=t+temp_tao(barOrder(i),:);
        te(i)=arrivalTime;
    end
end
end
function [TimeAdded] = addBarTime(bar, barIndex, addTime)
global TrackbarsPoints nk
TimeAdded = zeros(1, nk); 
nonZeroValues = TrackbarsPoints(barIndex, TrackbarsPoints(barIndex,:)~=0); 
TimeAdded(nonZeroValues) = addTime;
end