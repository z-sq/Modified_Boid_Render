function [totalCollisions, exchangeTriggered, twoColliding, multipleColliding, stepsForPtCld] = renderPointClouds(illumiToDispCellRatio, ratioNum)

display = 0;

showPointCloud = 0;
% fileNames = ["butterfly.csv", "cat.csv","teapot.csv"];
% fileNames = ["./cat_114.csv", "./teapot_100.csv", "./butterfly_94.csv"];
% fileNames = ["pt1619.1727.ptcld", "pt1630.1562.ptcld", "pt1617.1197.ptcld", "pt1620.997.ptcld", "pt1625.760.ptcld", "pt1608.758.ptcld", "pt1609.454.ptcld"];

fileNames = [ "pt1625.760.ptcld", "pt1608.758.ptcld", "pt1609.454.ptcld"];
% fileNames = ["pt1605_change.ptcld","pt1709_change.ptcld","pt1811_change.ptcld","pt1547_change.ptcld", "pt1379_change.ptcld"];

iterations = 1;

dispatcherPos = [[0, 0, 0];[0, 120 ,0];[120, 0, 0]; [120, 120 ,0]];
maxSpeed = 3;
maxAcc = 3;
timeunit = 1/20;
checkSteps = ((maxSpeed/maxAcc)/timeunit) + 1;
% checkSteps = 1;
dispCellRadius = 0.2;
% illumiToDispCellRatio = 5;
launchPerSec = 6;
radioRange = 10;

checkIllumCells = 3.5;

if illumiToDispCellRatio * checkIllumCells * dispCellRadius <= 0.5 * maxSpeed^2/maxAcc + dispCellRadius
    checkIllumCells = (0.5 * maxSpeed^2/maxAcc + dispCellRadius)/(illumiToDispCellRatio * dispCellRadius);
end

illuminationCellRadius = dispCellRadius * illumiToDispCellRatio;

pointCloud = convertCellListToMat("./pointclouds/" + fileNames(1));

% pointCloud = readmatrix("./pointclouds/" + fileNames(1));

% Implement Min Dist Algorithm
marginCount = zeros(1,4);
assignedToDispatcher = zeros(size(pointCloud,1), size(pointCloud,2),size(dispatcherPos,1));
for i = 1 : size(pointCloud,1)
    dist = inf;
    dispatcherIndex = 0;
    for j = 1 : size(dispatcherPos,1)
        distToDispatcher = norm(pointCloud(i,:)-dispatcherPos(j,:));
        if distToDispatcher < dist
            dist = distToDispatcher;
            dispatcherIndex = j;
        end
    end

    marginCount(dispatcherIndex) = marginCount(dispatcherIndex) + 1;
    assignedToDispatcher(marginCount(dispatcherIndex),:,dispatcherIndex) = pointCloud(i,:);
end

displayPlotSize = max(pointCloud, [],'all') * 1.2;

boidsNum = size(pointCloud,1);

boidsDispatched = 0;

if display
    % Iniialize the boids with coordinate, and velocity.
    h = plot3(0,0,0);
    hold on;
    xlim([0, displayPlotSize]);
    ylim([0, displayPlotSize]);
    zlim([0, displayPlotSize]);
    arrows = [];

    for i = 1 : boidsNum
        arrows(i) = arrow('Start',[-100,-100,-100],'Stop',[-100,-100,-100],'Length',0,'BaseAngle',0);
    end
end

step = 0;

totalCollisions = 0;
unWanteseCollisions = 0;

twoColliding = 0;
multipleColliding = 0;

exchangeTriggered = 0;


lastStepPos = zeros(boidsNum, 3);
currentStepPos = zeros(boidsNum, 3);

recoverAvoidance = [];

positionTypeNames = ["Ahead", "side", "side", "side", "side", "Behind"];
positionTypes = [];

exchangeInfo = [];

stepsForPtCld = [];

distance = [];

distPerPtCld = [];

renderInfo = [];

for iterate = 1 : iterations
    for ptCld = 1 : length(fileNames)

%         pointCloud = readmatrix("./pointclouds/" + fileNames(ptCld));
        pointCloud = convertCellListToMat("./pointclouds/" + fileNames(ptCld));
        step = 0;
        arrivedInfo = zeros(boidsNum,3);

        % clear all arrived information
        arrived = zeros(1,boidsNum);
        arrivedNum = 0;
        
        boidsNumRequired = size(pointCloud,1);

        lastTimeGoToTarget = zeros(boidsNum,1);

        if boidsDispatched == boidsNum
            for i = 1 : boidsNumRequired
                if boids(i).removed
                    arrived(i) = 1;
                    arrivedNum = arrivedNum + 1;
                    boids(i).startPt = boids(i).position;
                    continue;
                end
                boids(i).target = pointCloud(i,:);
                boids(i).startPt = boids(i).position;
                boids(i).arrived = false;
                boids(i).distTraveled = 0;
                boids(i).direction = [0,0,0];
                boids(i).plan = zeros(checkSteps, 6);
            end

            for i = boidsNumRequired + 1 : boidsNum
                if boids(i).removed
                    arrived(i) = 1;
                    arrivedNum = arrivedNum + 1;
                    boids(i).startPt = boids(i).position;
                    continue;
                end
                boids(i).startPt = boids(i).position;
                boids(i).goDark = true;
                boids(i).arrived = true;
                boids(i).distTraveled = 0;
                arrived(i) = 1;

                boids(i).direction = [0,0,0];
                boids(i).plan = zeros(checkSteps, 6);
                if display
                    arrows(i) = arrow(arrows(i),'Start',boids(i).position,'Stop',boids(i).position,'Length',3,'BaseAngle',20, 'Color', [0.2, 0.2, 0.2]);
                end
            end
        end

        speeds = zeros(boidsNum,4);

        while ~all(arrived)

            step = step + 1;
%             if step > 100
%                 disp(boids(89).direction);
%                 disp(boids(89).speed);
%             end

            % at beginning, generate FLSs from dispatcher
            if boidsDispatched < boidsNum && ~mod(step-1, 1/launchPerSec/timeunit)
                for dispatcher = 1 : size(dispatcherPos, 1)

                    if ~marginCount(dispatcher)
                        continue;
                    end
                    newBoidID = boidsDispatched + 1;
                    if newBoidID > boidsNum
                        break;
                    end
                    boids(newBoidID) = Boid(newBoidID, dispatcherPos(dispatcher,:), maxSpeed, maxAcc, checkSteps, timeunit, dispCellRadius, radioRange, illuminationCellRadius);
                    boids(newBoidID).target = assignedToDispatcher(marginCount(dispatcher), :, dispatcher);
                    boids(newBoidID).startPt = [-100, -100, -100];
                    boids(newBoidID).speed = 0;
%                     boids(newBoidID).direction = (boids(newBoidID).target - dispatcherPos(dispatcher,:))/norm(boids(newBoidID).target - dispatcherPos(dispatcher,:));
                    lastTimeGoToTarget(newBoidID) = step - 1;
                    boidsDispatched = boidsDispatched + 1;
                    marginCount(dispatcher) =  marginCount(dispatcher) - 1;
                end
            end

            for i = 1 : boidsDispatched
                if boids(i).removed || boids(i).arrived
                    continue;
                end

                for j = 1 : boidsDispatched
                    
                    if (step == 276 || step == 275 || step == 274) && i== 50 && j == 42
                        pause(0.1);
                    end

                    if ~boids(j).removed && boids(j).arrived
                        distToObstacle = norm(boids(i).position - boids(j).position);
                        clappingAngle = calculateClappingAngle((boids(j).position - boids(i).position), boids(i).direction);
                        if distToObstacle < radioRange && ...
                                abs(clappingAngle) <= pi/2 && ...
                                (sin(abs(clappingAngle)) * distToObstacle) < boids(i).dispCellRadius && ...
                                norm(boids(j).position - boids(i).position) < checkIllumCells * illuminationCellRadius

                            tmp = boids(i).target;
                            boids(i).target = boids(j).target;
                            boids(j).target = tmp;

                            % change go dark
                            tmp = boids(i).goDark;
                            boids(i).goDark = boids(j).goDark;
                            boids(j).goDark = tmp;

                            arrived(j) = false;
                            boids(j).arrived = false;

                            arrivedNum = arrivedNum - 1;

%                             if i < j
%                                 boids(i).exchangingWith = j;
%                             else
%                                 boids(j).exchangingWith = i;
%                             end

                            exchangeTriggered = exchangeTriggered + 1;
                            exchangeInfo = [exchangeInfo; i, j, boids(i).position, boids(i).target];

                            fprintf("Drone %d and %d exchange target\n", i, j);
                        end
                    end
                end
            end


            for i = 1 : boidsDispatched
                if boids(i).removed || boids(i).arrived
                    lastTimeGoToTarget(i,1) = step;
                    
                    continue;
                end
                if step >= 273 && step <= 277 && i == 42
                    pause(0.1);
                end

                [boids(i), avoidingType, positionType, cantAvoid] = boids(i).planMove(boids);
                if cantAvoid
                    arrived(i) = 1;
                    boids(i).arrived = true;
                    boids(i).speed = 0;
                    arrivedNum = arrivedNum + 1;

                    arrivedInfo(i,:) = [step, boids(i).distTraveled, (boids(i).distTraveled)/step];
                    continue;
                end

                if avoidingType == 1
                    twoColliding = twoColliding + 1; 
                    positionTypes = [positionTypes;positionTypeNames(positionType), i, step];
                elseif avoidingType == 2
                    multipleColliding = multipleColliding + 1;
                elseif avoidingType == 0 
                    if (step - lastTimeGoToTarget(i,1) - 1)
                        stepToRecover = step - lastTimeGoToTarget(i,1) - 1;
                        recoverAvoidance = [recoverAvoidance; stepToRecover, i , lastTimeGoToTarget(i,1), step];
                    end

                    lastTimeGoToTarget(i,1) = step;
                end

            end

            for i = 1 : boidsDispatched
                if boids(i).removed || boids(i).arrived
                    continue;
                end


                lastStepPos(i,:) = boids(i).position;

                [boids(i), arrivedSignal] = boids(i).makeMove();
                if arrivedSignal
                    arrived(i) = 1;
                    arrivedNum = arrivedNum + 1;
                    arrivedInfo(i,:) = [step, boids(i).distTraveled, (boids(i).distTraveled)/step];
                    boids(i).direction = [0,0,0];
%                
                end

                speeds(i,1) = max(speeds(i,1), boids(i).speed);

                speeds(i,4) = speeds(i,4) + 1;
                speeds(i,2) = ((speeds(i,4) - 1) * speeds(i,2) + boids(i).speed) / speeds(i,4);
                speeds(i,3) = min(speeds(i,3), boids(i).speed);

                currentStepPos(i,:) = boids(i).position;


                % Check if arrived illumination cell
%                 if abs(norm(boids(i).position - boids(i).target)) < illuminationCellRadius && ~arrived(i)
%                     arrived(i) = 1;
%                     boids(i).arrived = true;
%                     boids(i).speed = 0;
%                     arrivedNum = arrivedNum + 1;
% 
%                     arrivedInfo(i,:) = [step, boids(i).distTraveled, (boids(i).distTraveled)/step];
%                 end

                % check if arrived at center of illumination cell
%                 if abs(norm(boids(i).position - boids(i).target)) < dispCellRadius
%                     boids(i).arrived = true;
%                 end

                % Check if collided
                for j = 1 : (i - 1)
                    if i == j || boids(j).removed
                        continue;
                    end

                    if abs(norm(boids(i).position - boids(j).position)) < dispCellRadius
                        
                        fprintf("Drone %d at [%.4f, %.4f, %.4f] collided with drone %d at [%.4f, %.4f, %.4f], reletive distance %f \n", i, boids(i).position, j, boids(j).position, norm(boids(i).position - boids(j).position));
                        totalCollisions = totalCollisions + 1;
                        if ~boids(j).arrived
                            unWanteseCollisions = unWanteseCollisions + 1;
                        end
                        arrived(i) = 1;
                        boids(i).removed = true;
                        boids(i).arrived = true;
                        boids(i).position = [-100, -100, -100];
                        arrivedNum = arrivedNum + 1;
                        if display
                            arrows(i) = arrow(arrows(i),'Start',boids(i).position,'Stop',boids(i).position,'Length',3,'BaseAngle',20, 'Color', 'b');
                        end
                    end
                end

                if display
                    if boids(i).removed
                        continue;
                    end

                    if arrived(i)
                        if boids(i).goDark
                            arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', [0.2, 0.2, 0.2]);

                        else
                            arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'r');
                        end
                    else
                        arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'b');
                    end
                end

            end
            if display
                pause(0.0000001);
            end
            fprintf("Iteration %d, rendering %s, step %d, %d arrived, %d collisions, %d are un-wanted, %d exchanges triggered, prevented %d two-boids-collision and %d multi-boid-collision\n", ...
                iterate, fileNames(ptCld), step, arrivedNum, totalCollisions, unWanteseCollisions, exchangeTriggered, twoColliding, multipleColliding);

        end

%         writematrix(arrivedInfo, "arrivedInfo.xlsx", 'Sheet',ptCld + iterate - 1);
        stepsForPtCld = [stepsForPtCld, step];

        for i = 1 : boidsNum
            distPerPtCld(i,:) = [boids(i).distTraveled];
        end

        distance = [distance, distPerPtCld];
        if showPointCloud
            displayPlotSize = max(pointCloud, [],'all') * 1.2;
            figure(j + 1);
            plot3(0,0,0);
            xlim([0, displayPlotSize]);
            ylim([0, displayPlotSize]);
            zlim([0, displayPlotSize]);
            hold on;
            
            for i = 1 : boidsDispatched
                if boids(i).removed
                    continue;
                end

                if boids(i).goDark
                    plot3(lastStepPos(i,1),lastStepPos(i,2),lastStepPos(i,3),'.','Color', [0.8, 0.8, 0.8]);
                    hold on;
                else
                   plot3(lastStepPos(i,1),lastStepPos(i,2),lastStepPos(i,3),'.','Color', 'r');
                   hold on;
                end
            end

            pause(0.0000001);
        end

        timeSpent = arrivedInfo(:, 1);

        renderInfo = [renderInfo,max(timeSpent), mean(timeSpent), min(timeSpent),...
            max(speeds(:,1)), mean(speeds(:,2)), min(speeds(:,3)), ...
            max(distPerPtCld), mean(distPerPtCld), min(distPerPtCld)];
    end
end
pause(0.01);
writematrix(renderInfo, "renderInfo_90_newSpeedModel.xlsx", 'Sheet', ratioNum);
% writematrix(distance, "distanceTraveled_90.xlsx", 'Sheet', ratioNum);

% writematrix(recoverAvoidance, "recoverFromAvoidance_90.xlsx", 'Sheet', ratioNum);
% writematrix(positionTypes, "positionTypes_90.xlsx", 'Sheet', ratioNum);
writematrix(exchangeInfo, "exchangeInfo_90_newSpeedModel.xlsx", 'Sheet', ratioNum);

end

