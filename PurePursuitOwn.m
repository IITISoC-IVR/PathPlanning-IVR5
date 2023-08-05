function [deltaCar, lastFoundIndexOpt, xLookAheadPoint, yLookAheadPoint] = fcn(xCar, yCar, psi, xTraj, yTraj, l_wb, lookAheadDist, lastFoundIndexPrev)
rear_axle_x = xCar - 0.5*l_wb*cos(psi);
rear_axle_y = yCar - 0.5*l_wb*sin(psi);
curr_x = rear_axle_x;
curr_y = rear_axle_y;
%foundIntersection = false;
lastFoundIndexOpt = lastFoundIndexPrev;
startingIndex = lastFoundIndexPrev;
%Line-Circle Intersection code:
for i = startingIndex:length(xTraj)
x1 = xTraj(i) - curr_x;
y1 = yTraj(i) - curr_y;
x2 = xTraj(i+1) - curr_x;
y2 = yTraj(i+1) - curr_y;
dx = x2-x1;
dy = y2-y1;
dr = sqrt(dx^2+dy^2);
D = x1*y2 - x2*y1;
discriminant = (lookAheadDist^2)*(dr^2) - D^2;
if discriminant>=0 % Solution exists
if dy>=0
sol_x1 = (D*dy + dx*sqrt(discriminant))/dr^2;
sol_x2 = (D*dy - dx*sqrt(discriminant))/dr^2;
sol_y1 = (-D*dx + dy*sqrt(discriminant))/dr^2;
sol_y2 = (-D*dx - dy*sqrt(discriminant))/dr^2;
else
sol_x1 = (D*dy - dx*sqrt(discriminant))/dr^2;
sol_x2 = (D*dy + dx*sqrt(discriminant))/dr^2;
sol_y1 = (-D*dx - dy*sqrt(discriminant))/dr^2;
sol_y2 = (-D*dx + dy*sqrt(discriminant))/dr^2;
end
%Vehicle frame to Global frame
sol_pt1 = [sol_x1 + curr_x, sol_y1 + curr_y];
sol_pt2 = [sol_x2 + curr_x, sol_y2 + curr_y];
%Line-circle intersection code ends(in the following code, we should have used global
%coordinates for min and max but in this case it does not make any
%difference)
min_x = min(x1, x2);
min_y = min(y1, y2);
max_x = max(x1, x2);
max_y = max(y1, y2);
% The solution should have x and y in the range of (min(x1, x2),
% max(x1, x2)), similar for the y coordinate.
% The code for which is below:
%if at least one solution is in range
if (((min_x <= sol_x1) && (sol_x1<= max_x)) && ((min_y <= sol_y1) && (sol_y1<= max_y))) || (((min_x <= sol_x2) && (sol_x2<= max_x)) && ((min_y <= sol_y2) && (sol_y2<= max_y)))
%foundIntersection = true;
%if both of them are in range, select the better one(whichever
% is close to the next waypoint)
if (((min_x <= sol_x1) && (sol_x1<= max_x)) && ((min_y <= sol_y1) && (sol_y1<= max_y))) && (((min_x <= sol_x2) && (sol_x2<= max_x)) && ((min_y <= sol_y2) && (sol_y2<= max_y)))
if sqrt((x2-sol_x1)^2 + (y2-sol_y1)^2) < sqrt((x2-sol_x2)^2 + (y2-sol_y2)^2)
goalPt = sol_pt1;
else
goalPt = sol_pt2;
end
%if only one is in range
elseif ((min_x <= sol_x1) && (sol_x1<= max_x)) && ((min_y <= sol_y1) && (sol_y1<= max_y))
goalPt = sol_pt1;
else
goalPt = sol_pt2;
end
if sqrt((x2-sol_x1)^2 + (y2-sol_y1)^2) < sqrt((x2-xCar)^2 + (y2-yCar)^2)
% update the last found index
lastFoundIndexOpt = i;
break
else
lastFoundIndexPrev = lastFoundIndexPrev + 1;
lastFoundIndexPrev = mod(lastFoundIndexPrev, 1500);
end
else % Select the waypoint as the goal point
%foundIntersection = false;
goalPt = [xTraj(lastFoundIndexPrev), yTraj(lastFoundIndexPrev)];
end
else
%foundIntersection = false;
goalPt = [xTraj(lastFoundIndexPrev), yTraj(lastFoundIndexPrev)];
end
end
%it is time to calculate the steering angle(Finally!!!)
e_ld = abs(goalPt(1) - curr_x);
k = 2*e_ld/(lookAheadDist)^2; % curvature of path
deltaCar = atan(k*l_wb); % steering angle
xLookAheadPoint = goalPt(1);
yLookAheadPoint = goalPt(2);
end

