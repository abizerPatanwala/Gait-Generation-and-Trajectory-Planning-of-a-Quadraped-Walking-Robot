function [fx, fy, Vfx, Vfy, time] = generateTrajectory(stride, zMax, timeStep, transferTime)
%numPoints = max(round((transferTime * 100) / 5.0), 100);
t0 = 0;
t5 = transferTime;
t1 = (t5 - t0) / 5.0 + t0;
t2 = (t5 - t0) / 5.0 + t1;
t3 = (t5 - t0) / 5.0 + t2;
t4 = (t5 - t0) / 5.0 + t3;
Vfx_max = (2*stride) / (t3 - t2 + t4-t1);
Vfy_max = (2*zMax) / (t2 - t0);
numPoints = ((t2 - t1) / timeStep);
time = [linspace(t0 , t1 - timeStep, numPoints); linspace(t1, t2 - timeStep, numPoints); 
        linspace(t2, t3 - timeStep, numPoints); linspace(t3, t4 - timeStep, numPoints);
        linspace(t4, t5 - timeStep, numPoints)];

Vfx = zeros(5, numPoints);
Vfy = zeros(5, numPoints);
fx = zeros(1,5*numPoints + 1);
fy = zeros(1,5*numPoints + 1);

for i = 1:numPoints
    Vfx(2,i) = (Vfx_max / (t2 - t1))*(time(2,i) - t1);
end
Vfx(3, :) = Vfx_max;

for i = 1:numPoints
    Vfx(4,i) = (Vfx_max / (t3 - t4))*(time(4,i) - t3) + Vfx_max;
end

for i = 1:numPoints
    Vfy(1,i) = (Vfy_max / (t1 - t0))*(time(1,i) - t0);
end

for i = 1:numPoints
    Vfy(2,i) = (Vfy_max / (t1 - t2))*(time(2,i) - t1) + Vfy_max;
end
for i = 1:numPoints
    Vfy(4,i) = (Vfy_max / (t3 - t4))*(time(4,i) - t3);
end
for i = 1:numPoints
    Vfy(5,i) = (Vfy_max / (t5 - t4))*(time(5,i) - t4) - Vfy_max;
end

for i = 1:5
    if i ~= 1
        fy((i-1)*numPoints + 1) = fy((i-1)*numPoints) + Vfy(i-1,end)*(time(i,1) - time(i-1,end));
        fx((i-1)*numPoints + 1) = fx((i-1)*numPoints) + Vfx(i-1,end)*(time(i,1) - time(i-1,end));
    end
    for j = 2:numPoints
        fx(j + (i-1)*numPoints) = fx(j + (i-1)*numPoints - 1) + Vfx(i,j-1)*(time(i,j) - time(i,j-1));
        fy(j + (i-1)*numPoints) = fy(j + (i-1)*numPoints - 1) + Vfy(i,j-1)*(time(i,j) - time(i,j-1));
    end
end
fx(end) = fx(end - 1) + Vfx(5,end)*(transferTime - time(5,end));
fy(end) = fy(end - 1) + Vfy(5,end)*(transferTime - time(5,end));
Vfx = reshape(Vfx', [1, numel(Vfx)]);
Vfx(1,end+1) = 0;
Vfy = reshape(Vfy', [1, numel(Vfy)]);
Vfy(1,end+1) = 0;

