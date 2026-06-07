clc;
clear;
close all;

%% MAP DEFINITION

mapWidth  = 10;
mapHeight = 40;

% Corridor boundaries
xmin = 0;
xmax = mapWidth;
ymin = 0;
ymax = mapHeight;

%% Obstacles
% [x y width height]

obstacles = [
    1.0  5.0  1.5  1.5;     % bottom-left obstacle
    4.0 15.0  2.0  8.0;     % middle obstacle
    5.0 28.0  2.0  2.0;     % upper-right obstacle
];

%% Start and Goal

robot = [5 1];        % bottom center
goal  = [5 39];       % top center

%% Parameters

stepSize = 0.25;

goalThreshold = 0.5;

maxIterations = 5000;

%% Fuzzy distances

nearDist = 2.5;
farDist  = 5.0;

%% Plot Map

figure;
hold on;
axis equal;

xlim([-1 11]);
ylim([-1 41]);

title('Fuzzy Robot Navigation');

% Draw corridor
rectangle( ...
    'Position',[xmin ymin mapWidth mapHeight], ...
    'EdgeColor','k', ...
    'LineWidth',2);

% Draw obstacles
for k = 1:size(obstacles,1)

    rectangle( ...
        'Position',obstacles(k,:), ...
        'FaceColor',[0.8 0.2 0.2]);

end

plot(goal(1),goal(2),'gp',...
    'MarkerSize',15,...
    'MarkerFaceColor','g');

robotPlot = plot(robot(1),robot(2),...
    'bo',...
    'MarkerSize',8,...
    'MarkerFaceColor','b');

pathX = robot(1);
pathY = robot(2);

%% MAIN LOOP

for iter = 1:maxIterations

    %% Goal Attraction

    vGoal = goal - robot;

    if norm(vGoal) ~= 0
        vGoal = vGoal/norm(vGoal);
    end

    %% Obstacle Repulsion

    vAvoid = [0 0];

    minDistance = inf;

    for i = 1:size(obstacles,1)

        obs = obstacles(i,:);

        ox = obs(1);
        oy = obs(2);
        ow = obs(3);
        oh = obs(4);

        % Closest point on obstacle

        closestX = max(ox,min(robot(1),ox+ow));
        closestY = max(oy,min(robot(2),oy+oh));

        closestPoint = [closestX closestY];

        diffVec = robot - closestPoint;

        dist = norm(diffVec);

        minDistance = min(minDistance,dist);

        if dist < farDist && dist > 0

            away = diffVec/dist;

            strength = (farDist-dist)/farDist;

            vAvoid = vAvoid + strength*away;

        end

    end

    %% FUZZY LOGIC

    if minDistance <= nearDist

        % Very close obstacle
        alpha = 0.2;
        beta  = 1.5;

    elseif minDistance <= farDist

        % Medium distance obstacle

        muNear = (farDist-minDistance)/(farDist-nearDist);

        muFar = 1-muNear;

        alpha = muFar*1.0 + muNear*0.2;
        beta  = muFar*0.2 + muNear*1.5;

    else

        % No obstacle influence

        alpha = 1.0;
        beta  = 0.0;

    end

    %% Combine Behaviors

    velocity = alpha*vGoal + beta*vAvoid;

    if norm(velocity) > 0
        velocity = velocity/norm(velocity);
    end

    %% Update Position

    robot = robot + stepSize*velocity;

    %% Keep Inside Corridor

    robot(1) = max(xmin+0.2,min(xmax-0.2,robot(1)));
    robot(2) = max(ymin+0.2,min(ymax-0.2,robot(2)));

    %% Save Path

    pathX(end+1) = robot(1);
    pathY(end+1) = robot(2);

    %% Update Plot

    set(robotPlot,...
        'XData',robot(1),...
        'YData',robot(2));

    plot(pathX,pathY,'b');

    drawnow;

    %% Goal Check

    if norm(robot-goal) < goalThreshold

        disp('Goal Reached!');
        break;

    end

end

%% Final Path

plot(pathX,pathY,...
    'b',...
    'LineWidth',2);

disp(['Iterations = ',num2str(iter)]);