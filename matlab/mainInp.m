global preparationDistance lane_width safety_gap...
    trackbarNum trackbarDis Bars TrackbarsPoints nk
load('trackbars');load('matlab');
lane_width=3.5;
preparationDistance=200;
safety_gap=0;
[~,nk]=size(Bars);
global temp_tao 
crossingInterval = 1.5;
mergingInterval = 1;
temp_tao=zeros(size(Bars,1),nk);
for i=1:size(Bars,1)
    [temp_tao(i,:)] = addBarTime(Bars(i,:),i,crossingInterval);
end
b=[1 2 5 6 9 10 13 14 3 4 7 8 11 12 15 16];
for i=1:108
    if ismember(i,b)
        temp_tao(temp_tao(:,i) ~= 0, i) = mergingInterval;
    end
end
global  speedInConflictZone ...
    max_acceleration max_deceleration avg_entry_speed...
    vehicle_width vehicle_length vs_avg
speedInConflictZone=30/3.6;
avg_entry_speed=60/3.6;
max_acceleration=2.0;max_deceleration=-3.0;
vehicle_width=1.8; vehicle_length=4.5;
vs_avg=60/3.5;
function [TimeAdded] = addBarTime(bar,barIndex,addTime)
global TrackbarsPoints nk
TimeAdded=zeros(size(bar,1),nk);
nonZeroValues = TrackbarsPoints(barIndex,TrackbarsPoints(barIndex,:)~=0);
TimeAdded(nonZeroValues)=addTime;
end
