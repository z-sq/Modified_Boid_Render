function renderPointClouds()
clear;
close all;
clc;

display = 1;
fileNames = ["./butterfly.csv", "./cat.csv","./teapot.csv"];
iterations = 1;

dispatcherPos = [0, 0, 0];
maxSpeed = 5;
checkSteps = 5;
timeunit = 1/25;
dispCellRadius = 0.2;
illuminationCellRadius = 0.5;
launchPerSec = 5;


for file = 1 : length(fileNames)
    pointClouds(:,:,file) = readmatrix(fileNames(file));
end

displayPlotSize = max(pointClouds, [],'all') * 1.2;

boidsNum = size(pointClouds,1);

if display
    % Iniialize the boids with coordinate, and velocity.
    h = plot3(0,0,0);
    hold on;
    xlim([0, displayPlotSize]);
    ylim([0, displayPlotSize]);
    zlim([0, displayPlotSize]);
    arrows = [];

    for i = 1 : boidsNum
        arrows(i) = arrow('Start',[0,0,0],'Stop',[0,0,0],'Length',0,'BaseAngle',0);
    end
end

boids = [];

step = 0;

totalCollisions = 0;
unWanteseCollisions = 0;


lastStepPos = zeros(boidsNum, 3);
currentStepPos = zeros(boidsNum, 3);

for iterate = 1 : iterations
    for ptCld = 1 : size(pointClouds, 3)


        % clear all arrived information
        arrived = zeros(1,boidsNum);
        arrivedNum = 0;

        if length(boids) == boidsNum
            for i = 1 : boidsNum
                if boids(i).removed
                    arrived(i) = 1;
                    arrivedNum = arrivedNum + 1;
                    continue;
                end
                boids(i).target = pointClouds(i,:,ptCld);
                boids(i).arrived = false;
            end
        end

        while ~all(arrived)
            step = step + 1;

            % at beginning, generate FLSs from dispatcher
            if length(boids) < boidsNum && ~mod(step-1, 1/launchPerSec/timeunit)
                newBoidID = size(boids, 2) + 1;
                boids = [boids, Boid(newBoidID, dispatcherPos, maxSpeed, checkSteps, timeunit, dispCellRadius)];
                boids(newBoidID).target = pointClouds(newBoidID,:,ptCld);
                boids(newBoidID).speed = maxSpeed;
                boids(newBoidID).direction = (boids(newBoidID).target - dispatcherPos)/norm(boids(newBoidID).target - dispatcherPos);
            end


            
            for i = 1 : size(boids, 2)
                if boids(i).removed || boids(i).arrived
                    continue;
                end

%                 if i == 28 && step == 524
%                     pause(0.01);
%                 end

                boids(i) = boids(i).planMove(boids);

            end

            for i = 1 : size(boids, 2)
                if boids(i).removed || boids(i).arrived
                    continue;
                end


                lastStepPos(i,:) = boids(i).position;

                boids(i) = boids(i).makeMove();

                currentStepPos(i,:) = boids(i).position;


                % Check if arrived illumination cell
                if abs(norm(boids(i).position - boids(i).target)) < illuminationCellRadius && ~arrived(i)
                    arrived(i) = 1;
%                     boids(i).arrived = true;
                    arrivedNum = arrivedNum + 1;
                end
                % check if arrived at center of illumination cell
                if abs(norm(boids(i).position - boids(i).target)) < dispCellRadius
                    boids(i).arrived = true;
                end

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
                        arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'r');
                    else
                        arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'b');
                    end
                end

            end
            if display
                pause(0.0000001);
            end
            fprintf("Iteration %d, rendering %s, step %d, %d has arrived, total %d collisions, %d are un-expected\n", iterate, fileNames(ptCld), step, arrivedNum, totalCollisions, unWanteseCollisions);

        end
    end
end

end

