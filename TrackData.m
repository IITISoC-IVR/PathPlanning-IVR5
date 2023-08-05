%% Tracks - Austin and Silverstone

%% Austin- Min Curv

if NTrack == 1
    X = AustinMinCurvature{:,1};
    Y = AustinMinCurvature{:,2};
    x = AustinBoundary{:,1};
    y = AustinBoundary{:,2};
    twr = Austin{:,3};
    twl = Austin{:,4};

    plot(X, Y, 'k');
    
end

%% Austin- Shortest Path
if NTrack == 2
    X = AustinShortest{:,1};
    Y = AustinShortest{:,2};
    x = AustinBoundary{:,1};
    y = AustinBoundary{:,2};
    twr = Austin{:,3};
    twl = Austin{:,4};

    plot(X, Y, 'k');
    
end

%% Silverstone- Min Curv

if NTrack == 3
    X = SilverstoneMinCurvature{:,1};
    Y = SilverstoneMinCurvature{:,2};
    x = SilverstoneBoundary{:,1};
    y = SilverstoneBoundary{:,2};
    twr = Silverstone{:,3};
    twl = Silverstone{:,4};

    plot(X, Y, 'k');
    
end

%% Silverstone- Shortest Path

if NTrack == 4
    X = SilverstoneShortestPath{:,1};
    Y = SilverstoneShortestPath{:,2};
    x = SilverstoneBoundary{:,1};
    y = SilverstoneBoundary{:,2};
    twr = Silverstone{:,3};
    twl = Silverstone{:,4};

    plot(X, Y, 'k');

end

%% Track Sides

lHalfTrackWidth = 2.5;

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

%% End

xStartLine=[X(1),X(1)];
yStartLine=Y(1)+1.5*[-lHalfTrackWidth,lHalfTrackWidth];

lTrack=0;
for i=2:length(X)
    lTrack=lTrack+sqrt((X(i)-X(i-1))^2+(Y(i)-Y(i-1))^2);
end

clear X Y 
