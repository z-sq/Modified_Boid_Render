function renderPointClouds()
clear;
close all;

boidsNum = 90;
fileNames = ["./butterfly.csv", "./cat.csv","./teapot.csv"];
iterations = 1;

dispatcherPos = [0, 0];
maxSpeed = 5;
checkSteps = 5;
timeunit = 1/25;
dispCellRadius = 0.2;
illuminationCellRadius = 1;
launchPerSec = 2;

% Iniialize the boids with coordinate, and velocity.
h = plot3(0,0,0);
hold on;
xlim([0, 100]);
ylim([0, 100]);
zlim([0, 100]);
arrows = [];

for i = 1 : boidsNum
    arrows(i) = arrow('Start',[0,0,0],'Stop',[0,0,0],'Length',0,'BaseAngle',0);
end

for file = 1 : length(fileNames)
    pointClouds(:,:,file) = readmatrix(fileNames(file));
end

boids = [];

step = 0;

lastStepPos = zeros(boidsNum, 2);
currentStepPos = zeros(boidsNum, 2);

for iterate = 1 : iterations
    for ptCld = 1 : size(pointClouds, 3)

        arrived = zeros(1,boidsNum);

        while ~all(arrived)
            step = step + 1;

            % at beginning, generate FLSs from dispatcher
            if length(boids) < 90 && ~mod(step, 1/launchPerSec/timeunit)
                newBoidID = size(boids, 2) + 1;
                boids = [boids, Boid(newBoidID, dispatcherPos, maxSpeed, checkSteps, timeunit, dispCellRadius)];
                boids(newBoidID).target = pointClouds(newBoidID,:,1);
                boids(newBoidID).speed = maxSpeed;
                boids(newBoidID).direction = (boids(newBoidID).target - dispatcherPos)/norm(boids(newBoidID).target - dispatcherPos);
            end


            
            for i = 1 : size(boids, 2)
                if arrived(i)
                    continue;
                end

                lastStepPos(i,:) = boids(i).position;

                boids(i) = boids(i).move(boids);

                currentStepPos(i,:) = boids(i).position;

                if abs(norm(boids(i).position - boids(i).target)) < illuminationCellRadius
                    arrived(i) = 1;
                    boids(i).arrived = true;
                end

            end

            for i = 1 : size(boids, 2)
                if arrived(i)
                    arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'r');
                else
                    arrows(i) = arrow(arrows(i),'Start',lastStepPos(i,:),'Stop',currentStepPos(i,:),'Length',3,'BaseAngle',20, 'Color', 'b');
                end
            end
            pause(0.00001);


        end
    end
end

end

