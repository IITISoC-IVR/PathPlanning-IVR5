function [trajMCP, trackData] = Trajectory_Optimizer(track,name)

%% Processing  track data

% track data - first point repeated
data = track;

% x,y and track width data 
x =  data(:,1);
y =  data(:,2);
twr = data(:,3);
twl = data(:,4);

% interpolate data to get finer curve with equal distances between each segment

% higher no. of segments causes trajectory to follow the reference line
nseg = 1500;

pathXY = [x y];
stepLengths = sqrt(sum(diff(pathXY,[],1).^2,2));
stepLengths = [0; stepLengths]; % add the starting point
cumulativeLen = cumsum(stepLengths);
finalStepLocs = linspace(0,cumulativeLen(end), nseg);
finalPathXY = interp1(cumulativeLen, pathXY, finalStepLocs);
xt = finalPathXY(:,1);
yt = finalPathXY(:,2);
twrt = interp1(cumulativeLen, twr, finalStepLocs,'spline')';
twlt = interp1(cumulativeLen, twl, finalStepLocs,'spline')';




% normal direction for each vertex
dx = gradient(xt);
dy = gradient(yt);
dL = hypot(dx,dy);

% offset curve - anonymous function
xoff = @(a) -a*dy./dL + xt;
yoff = @(a)  a*dx./dL + yt;

% plot reference line
plot(xt,yt,'g')
hold on

% offset data
offset = [-twrt twlt];
for i = 1:numel(xt)
    xin = xoff(offset(i,1));      % get inner offset curve
    yin = yoff(offset(i,1));
    
    xout  = xoff(offset(i,2));      % get outer offset curve
    yout  = yoff(offset(i,2));
end

% plot inner track
plot(xin,yin,'color','b','linew',2)
% plot outer track
plot(xout,yout,'color','r','linew',2)
hold off
axis equal

xlabel('x(m)','fontweight','bold','fontsize',14)
ylabel('y(m)','fontweight','bold','fontsize',14)
title(sprintf(name),'fontsize',16)

% form delta matrices
delx = xout - xin;
dely = yout - yin;

trackData = [xt yt xin yin xout yout];

% number of segments
n = numel(delx);

%% Shortest trajectory

% preallocation
H_S = zeros(n);
B_S = zeros(size(delx)).';

% formation of H_S matrix (nxn)
for i=1:n-1
    
    H_S(i,i)     = H_S(i,i)     + delx(i)^2          + dely(i)^2;
    H_S(i+1,i)   = H_S(i+1,i)   - delx(i)*delx(i+1)  - dely(i)*dely(i+1);
    H_S(i,i+1)   = H_S(i,i+1)   - delx(i)*delx(i+1)  - dely(i)*dely(i+1);
    H_S(i+1,i+1) = H_S(i+1,i+1) + delx(i+1)^2        + dely(i+1)^2;
    
end

% formation of B matrix (1xn)
for i=1:n-1
    B_S(1,i)   = B_S(1,i)   - 2*(xin(i+1)-xin(i))*delx(i)   - 2*(yin(i+1)-yin(i))*dely(i);
    B_S(1,i+1) = B_S(1,i+1) + 2*(xin(i+1)-xin(i))*delx(i+1) + 2*(yin(i+1)-yin(i))*dely(i+1);
end

%% Minimum Curvature

% preallocation
H_C = zeros(n);
B_C = zeros(size(delx)).';

% formation of H_C matrix (nxn)
for i=2:n-1
    
    % first row
    H_C(i-1,i-1) = H_C(i-1,i-1) + delx(i-1)^2         + dely(i-1)^2;
    H_C(i-1,i)   = H_C(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
    H_C(i-1,i+1) = H_C(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
    
    %second row
    H_C(i,i-1)   = H_C(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
    H_C(i,i)     = H_C(i,i )    + 4*delx(i)^2         + 4*dely(i)^2;
    H_C(i,i+1)   = H_C(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
    
    % third row
    H_C(i+1,i-1) = H_C(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
    H_C(i+1,i)   = H_C(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
    H_C(i+1,i+1) = H_C(i+1,i+1) + delx(i+1)^2         + dely(i+1)^2;
    
end

% formation of B_C matrix (1xn)
for i=2:n-1
    
    B_C(1,i-1) = B_C(1,i-1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i-1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i-1);
    B_C(1,i)   = B_C(1,i)   - 4*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i)   - 4*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i);
    B_C(1,i+1) = B_C(1,i+1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i+1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i+1);
    
end

%% rTrajOpt value
rTrajOpt = 0.005;

%% Problem Formulation

H=rTrajOpt*H_S+(1-rTrajOpt)*H_C;

B=rTrajOpt*B_S+(1-rTrajOpt)*B_C;



%% quadprog

% define constraints
lb = zeros(n,1);
ub = ones(size(lb));

% if start and end points are the same
Aeq      =   zeros(1,n);
Aeq(1)   =   1;
Aeq(end) =   -1;
beq      =   0;
    
%% Solver

options = optimoptions('quadprog','Display','iter');
[resMCP,fval,exitflag,output] = quadprog(2*H,B',[],[],Aeq,beq,lb,ub,[],options);

%% Plotting results

% co-ordinates for the resultant curve
xresMCP = zeros(size(xt));
yresMCP = zeros(size(xt));

for i = 1:numel(xt)
    xresMCP(i) = xin(i)+resMCP(i)*delx(i);
    yresMCP(i) = yin(i)+resMCP(i)*dely(i);
end

% plot minimum curvature trajectory
figure
plot(xresMCP,yresMCP,'color','r','linew',1)
hold on

% plot starting line
plot([xin(1) xout(1)], [yin(1) yout(1)],'color','b','linew',2)
% plot([xin(2) xout(2)], [yin(2) yout(2)],'color','k','linew',2)

% plot reference line
plot(xt,yt,'--')
hold on

% plot inner track
plot(xin,yin,'color','k')

%plot outer track
plot(xout,yout,'color','k')
hold off
axis equal

xlabel('x(m)','fontweight','bold','fontsize',14)
ylabel('y(m)','fontweight','bold','fontsize',14)
title(sprintf(name,'%s - Minimum Curvature Trajectory'),'fontsize',16)

trajMCP = [xresMCP yresMCP];
