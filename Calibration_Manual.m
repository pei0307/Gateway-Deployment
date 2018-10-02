function [R,C,floorPlanBW] = Calibration_Manual(floorPlanBW)
% Optional delation of the image to make wall selection easier

figure
imshow(floorPlanBW,'InitialMagnification',100);
title('Floor Plan');
disp('Select two points from walls to calibrate the plan!');
while 1
    disp('Select the 1st point')
    try
        [r,c] = ginput(1); % get points one by one. Check each to be a- hit not miss
        R(1,1)= round(r);
        C(1,1) = round(c);

    catch
    end
    if ~floorPlanBW(C(1,1),R(1,1))
        break % if its a hit it stops
    end
end

while 1
    disp('Select the 2nd point')
    [r,c] = ginput(1); % get points one by one. Check each to be a- hit not miss
    R(2,1) = round(r)
    C(2,1) = round(c)

    if ~floorPlanBW(C(2,1),R(2,1))
        break % if its a hit it stops
    end
end
end