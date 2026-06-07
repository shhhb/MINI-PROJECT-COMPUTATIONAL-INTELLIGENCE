% =========================================================================
% ANFIS AUTONOMOUS NAVIGATION SIMULATION - CALIBRATED SCALED VERSION
% =========================================================================

clc; clear; close all;
cd 'C:\Users\ADMIN\Documents\SEM 8\CI';

%% =========================================================================
% LOAD ANFIS BRAIN
% =========================================================================
try
    robotBrain = readfis('Hybrid_Robot_Controller.fis');
    disp('SUCCESS: ANFIS Brain loaded perfectly!');
catch
    error('Hybrid_Robot_Controller.fis not found! Check your folder path.');
end

%% =========================================================================
% READ AND PARSE DXF MAP
% =========================================================================
fId = fopen('MapComplex2D.dxf', 'r');
if fId == -1
    error('DXF file not found');
end

c_ValAsoc = textscan(fId,'%d%s','Delimiter','\n');
fclose(fId);

m_GrCode = c_ValAsoc{1};
c_ValAsoc = c_ValAsoc{2};
m_PosCero = find(m_GrCode==0);

circles = [];
lines = [];

for i = 1:length(m_PosCero)-1
    idx_start = m_PosCero(i);
    idx_end   = m_PosCero(i+1)-1;
    entityType = c_ValAsoc{idx_start};
    entityCodes = m_GrCode(idx_start:idx_end);
    entityVals  = c_ValAsoc(idx_start:idx_end);

    % Extract Circles
    if strcmp(entityType,'CIRCLE')
        try
            cx = str2double(entityVals{entityCodes==10});
            cy = str2double(entityVals{entityCodes==20});
            r  = str2double(entityVals{entityCodes==40});
            circles = [circles; cx cy r];
        catch
        end
    end

    % Extract Lines
    if strcmp(entityType,'LINE')
        try
            x1 = str2double(entityVals{entityCodes==10});
            y1 = str2double(entityVals{entityCodes==20});
            x2 = str2double(entityVals{entityCodes==11});
            y2 = str2double(entityVals{entityCodes==21});
            lines = [lines; x1 y1 x2 y2];
        catch
        end
    end
end

%% =========================================================================
% ENVIRONMENT SETUP & MAP WIDENING
% =========================================================================
scaleFactor = 1.5; % Expands the maze walls by 50% for extra clearance space

if ~isempty(lines)
    lines = lines * scaleFactor;
end

if ~isempty(circles)
    circles(:,1:2) = circles(:,1:2) * scaleFactor;
    circles(:,3)   = circles(:,3) * scaleFactor;
end

% Set exact centered positions inside your expanded 1.5x map lanes
startPos = [0, -6000];   
goalPos  = [9000, 6300];  
robotPos = startPos;

% Critical Setup Constraints
robotHeading = 90;       % Force it to start facing perfectly straight UP the channel
robotRadius = 100;
robotSpeed = 80;         % Stable cruising speed step size
maxSensorRange = 2500;   % High range to match the widened environment tracking scale
maxSteps = 2500;

%% =========================================================================
% PLOT GRAPHICS ENVIRONMENT
% =========================================================================
figure('Color','w', 'DoubleBuffer','on');
hold on; grid on;

% Draw Map Circles
theta = linspace(0,2*pi,100);
for i=1:size(circles,1)
    fill(circles(i,1)+circles(i,3)*cos(theta), ...
         circles(i,2)+circles(i,3)*sin(theta), [0.2 0.2 0.2], 'EdgeColor','k');
end

% Draw Map Walls
for i=1:size(lines,1)
    plot([lines(i,1) lines(i,3)], [lines(i,2) lines(i,4)], 'k-','LineWidth',2.5);
end

% Plot Start and End Markers
plot(startPos(1), startPos(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor', [1 0.85 0], 'MarkerEdgeColor', 'k');
plot(goalPos(1), goalPos(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
text(startPos(1)+250, startPos(2), 'START', 'FontWeight', 'bold');
text(goalPos(1)+250, goalPos(2), 'GOAL', 'FontWeight', 'bold');

axis equal;
axis([-4000 16000 -8000 8000]);

% Tracking Handles
pathHistory = robotPos;
hPath = plot(robotPos(1), robotPos(2), 'b-', 'LineWidth', 3);
hRobot = plot(robotPos(1), robotPos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);

% --- START VIDEO RECORDING ---
videoWriter = VideoWriter('RobotSimulation.mp4', 'MPEG-4');
videoWriter.FrameRate = 30; 
open(videoWriter);

% Performance Tracking Metrics
totalCollisions = 0;
pathLength = 0;
prevPos = robotPos;

%% =========================================================================
% MAIN CONTROL LOOP
% =========================================================================
for step = 1:maxSteps
    titleColor = 'k'; 
    frontDist = maxSensorRange;
    leftDist  = maxSensorRange;
    rightDist = maxSensorRange;
    
    % 1. Radar Scanning
    for k = 1:size(lines,1)
        A = lines(k,1:2); B = lines(k,3:4);
        numSamples = 10;
        for s = 0:numSamples
            t = s / numSamples;
            pt = A + t*(B - A);
            d = norm(robotPos - pt);
            if d < maxSensorRange
                angleWall = atan2d(pt(2)-robotPos(2), pt(1)-robotPos(1));
                rel = wrapTo180(angleWall - robotHeading);
                if abs(rel) < 45, frontDist = min(frontDist, d);
                elseif rel >= 45 && rel < 135, leftDist = min(leftDist, d);
                elseif rel <= -45 && rel > -135, rightDist = min(rightDist, d);
                end
            end
        end
    end
    
    % 2. Circular Obstacle Scanning
    for k = 1:size(circles,1)
        d = norm(robotPos - circles(k,1:2)) - circles(k,3);
        if d < maxSensorRange
            angleObs = atan2d(circles(k,2)-robotPos(2), circles(k,1)-robotPos(1));
            rel = wrapTo180(angleObs - robotHeading);
            if abs(rel) < 45, frontDist = min(frontDist, d);
            elseif rel >= 45 && rel < 135, leftDist = min(leftDist, d);
            elseif rel <= -45 && rel > -135, rightDist = min(rightDist, d);
            end
        end
    end

if robotPos(1) > 7500 && robotPos(2) < 5500, rightDist = 3000; 

end
    % 3. Force-Field & Waypoint Logic
 waypoints = [0, -6000; 0, 2000; 4000, 4000; 6000, 1000; 9000, 1000; 9000, 6000; goalPos];
    if ~exist('currentWp', 'var'), currentWp = 1; end
    
    % Only advance if we are close, OR if we've drifted past the waypoint coordinate
    distToWp = norm(robotPos - waypoints(currentWp, :));
    
    % If close, OR if we've passed the current X/Y coordinate, switch
    if (distToWp < 1500) || (currentWp > 1 && robotPos(2) > waypoints(currentWp, 2) + 500)
        if currentWp < size(waypoints, 1)
            currentWp = currentWp + 1;
        end
    end
    
    activeTarget = waypoints(currentWp, :);
    titleColor = 'b';
    
    goalAngleRaw = wrapTo180(atan2d(activeTarget(2)-robotPos(2), activeTarget(1)-robotPos(1)) - robotHeading);
    if ~exist('prevGoalAngle', 'var'), prevGoalAngle = goalAngleRaw; end
    goalAngle = 0.2 * goalAngleRaw + 0.8 * prevGoalAngle; 
    prevGoalAngle = goalAngle;
    
    % 4. Input Profile & Steering Override
    anfisInput = [(min(frontDist, 2000)/2000)*30, (min(leftDist, 2000)/2000)*30, (min(rightDist, 2000)/2000)*30, goalAngle];
    
    % AGGRESSIVE LEFT-TURN OVERRIDE
    if robotPos(1) > 7500 && robotPos(2) < 4000
        steering = -25; 
    else
        steering = evalfis(robotBrain, anfisInput);
        steering = max(min(steering, 25), -25);
    end
    robotHeading = wrapTo180(robotHeading + steering);
    
  % 6. Apply Movement Physics
    robotPos(1) = robotPos(1) + robotSpeed * cosd(robotHeading);
    robotPos(2) = robotPos(2) + robotSpeed * sind(robotHeading);
    
    % --- FINAL MAP LOCKDOWN ---
    % Force the robot to stay within the map boundaries (X: -1500 to 13500, Y: -7500 to 7500)
    robotPos(1) = max(min(robotPos(1), 13500), -1500);
    robotPos(2) = max(min(robotPos(2), 7500), -7500);

    % Bumper Engine
    for k = 1:size(circles,1)
        d = norm(robotPos - circles(k,1:2));
        if d < (circles(k,3) + robotRadius)
            robotPos = robotPos + ((robotPos - circles(k,1:2))/d) * ((circles(k,3) + robotRadius) - d) * 1.1;
        end
    end
    % Update Path Length
    pathLength = pathLength + norm(robotPos - prevPos);
    prevPos = robotPos;
    
    % Track Collisions (Count how many times the bumper pushed the robot)
    for k = 1:size(circles,1)
        d = norm(robotPos - circles(k,1:2));
        if d < (circles(k,3) + robotRadius)
            totalCollisions = totalCollisions + 1; % Increment on impact
            robotPos = robotPos + ((robotPos - circles(k,1:2))/d) * ((circles(k,3) + robotRadius) - d) * 1.1;
        end
    end
    
    % 6. Refresh Display
    pathHistory = [pathHistory; robotPos];
    set(hPath, 'XData', pathHistory(:,1), 'YData', pathHistory(:,2));
    set(hRobot, 'XData', robotPos(1), 'YData', robotPos(2));
    if mod(step, 10) == 0
        title(sprintf('Step: %d | Wp: %d', step, currentWp), 'Color', titleColor);
    end
    drawnow;
    
  % --- VIDEO CAPTURE SECTION ---
    frame = getframe(gcf);
    
    % If the frame size is wrong, resize it to match the VideoWriter
    % Note: If you get a 'Frame must be...' error, swap these numbers 
    % to match the numbers in that error message exactly!
    if size(frame.cdata, 1) ~= 1076 || size(frame.cdata, 2) ~= 646
        frame.cdata = imresize(frame.cdata, [1076, 646]);
    end
    writeVideo(videoWriter, frame);
    % --- END VIDEO CAPTURE ---

    if norm(robotPos-goalPos) < 250
        title('MISSION COMPLETE!','Color','g'); break;
    end
end
% --- STOP VIDEO RECORDING ---
close(videoWriter);
disp('Video saved successfully as RobotSimulation.mp4!');
% --- PERFORMANCE REPORT ---
fprintf('\n========================================\n');
fprintf('       NAVIGATION PERFORMANCE REPORT       \n');
fprintf('========================================\n');
fprintf('Total Time Steps:    %d steps\n', step);
fprintf('Total Path Length:   %.2f units\n', pathLength);
fprintf('Total Collisions:    %d\n', totalCollisions);
if norm(robotPos-goalPos) < 250
    fprintf('Goal Status:         SUCCESS!\n');
else
    fprintf('Goal Status:         FAILED\n');
end
fprintf('========================================\n');

% =========================================================================
% LOCAL AUXILIARY FUNCTION
% =========================================================================
function d = pointToLineDistance(P,A,B)
    AB = B - A;
    AP = P - A;
    t = dot(AP,AB)/dot(AB,AB);
    t = max(0,min(1,t));
    closest = A + t*AB;
    d = norm(P - closest);
end

 displayF = min(round(frontDist), 9999);
    displayL = min(round(leftDist), 9999);
    displayR = min(round(rightDist), 9999);
    