classdef Boid
    %BOID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID = 0;
        maxSpeed = 0;
        direction = [0,0];
        speed = 0;
        checkSteps = 0;
        position = [0,0]
        timeUnit = 0;
        dispCellRadius = 0;
        avoidAngel = pi/4;
        target = [0,0];
        arrived = false;
        removed = false;
    end
    
    methods
        function obj = Boid(ID, initialPosition, maxSpeed, checkSteps, timeunit, dispCellRadius)
            %BOID Construct an instance of this class
            %   Detailed explanation goes here
            obj.ID = ID;
            obj.maxSpeed = maxSpeed;
            obj.speed = obj.maxSpeed;
            obj.checkSteps = checkSteps;
            obj.position = initialPosition;
            obj.timeUnit = timeunit;
            obj.dispCellRadius = dispCellRadius;
        end

        function obj = planMove(obj, boids)
            % try to go to target
            obj = obj.goToTarget();

            % update direction change to boids
            boids(obj.ID) = obj;

            % if there are collisions, start avoiding
            collisions = obj.checkCollision(boids);
            if collisions
                obj = obj.avoidCollisions(boids, collisions);
            end
        end

        function obj = makeMove(obj)
            obj.position = obj.position + obj.getVelocity() * obj.timeUnit;
        end
        
        %   Rule 1: Avoid Collisions
        function obj = avoidCollisions(obj,boids, collisions)
            
            % Only 2 Boids colliding
            if size(collisions,1) == 1
                positionChoose = [];

               % if just 2 Boids colliding
                positionChoose = [positionChoose; obj.getNeighborPos(collisions(3:4))];

                % keep finding until no more collisions
                stillColliding = true;
                i = 0;
                while stillColliding
                    i = i + 1;
                    if ~any(positionChoose)
                        continue;
                    end

                    obj.direction = (positionChoose(i,:) - obj.position)/norm((positionChoose(i,:) - obj.position));
                    boids(obj.ID).direction = obj.direction;

                    recheckCollisions =  obj.checkCollision(boids);

                    % if no collisions, quit; if still collisions, add the
                    % place and
                    if size(recheckCollisions,1) == 0
                        stillColliding = false;
                    elseif size(recheckCollisions,1) >= 1
                        newPosibleChoose = obj.getNeighborPos(positionChoose(i,:));
                        positionChoose = [positionChoose; newPosibleChoose];
                        positionChoose = unique(positionChoose, 'rows', "stable");
                    end

                end

            elseif size(collisions, 1) > 1
                % if multiple Boids Collides, find the one with minimum ID
                % to be the leading boid
                leadingBoidID = min(collisions(:,1));

                transitionMaxis1 = [cos(obj.avoidAngel), sin(obj.avoidAngel);-sin(obj.avoidAngel), cos(obj.avoidAngel)];
                transitionMaxis2 = [cos(2*pi - obj.avoidAngel), sin(2*pi - obj.avoidAngel);-sin(2*pi - obj.avoidAngel), cos(2*pi - obj.avoidAngel)];
                % for all other boids, rotate their direction to form
%                 for i = 1 : size(collisions,1)
%                     collideBoidID = collisions(i,1);
%                     if collisions(i, 1) == leadingBoidID
%                         collideBoidID = obj.ID;
%                     end
%                     clappingAngle = atan2d(det([boids(leadingBoidID).direction;boids(collideBoidID).direction]),dot(boids(leadingBoidID).direction,boids(collideBoidID).direction));
%                     if clappingAngle <= 180
%                         boids(collideBoidID).direction = boids(leadingBoidID).direction * transitionMaxis1;
%                     else
%                         boids(collideBoidID).direction = boids(leadingBoidID).direction * transitionMaxis2;
%                     end
% 
%                 end

                clappingAngle = atan2d(det([boids(leadingBoidID).direction;obj.direction]),dot(boids(leadingBoidID).direction,obj.direction));
                if clappingAngle <= 180
                    obj.direction = boids(leadingBoidID).direction * transitionMaxis1;
                else
                    obj.direction = boids(leadingBoidID).direction * transitionMaxis2;
                end
            end
        end

        % Rule 2: go To Target
        function obj = goToTarget(obj)
            obj.direction = (obj.target - obj.position)/norm(obj.target - obj.position);
        end



        %   This method is used to check collision with other boids
        function collisions = checkCollision(obj, boids)

            collisions = [];
            collidingTimes = 0;

            posAtStep = zeros(length(boids), 2);
            % load the position of all boids
            for i = 1 : length(boids)
                posAtStep(i,:) = boids(i).position;
%                 fprintf("Drone %d at [%.2f, %.2f]\n", i, posAtStep(i, :));
            end

            for step = 1 : obj.checkSteps
                % update the position
                for i = 1 : length(boids)
                    posAtStep(i, :) = posAtStep(i,:) + boids(i).getVelocity() * boids(i).timeUnit;
                        
%                     fprintf("Drone %d predict after step %d at [%.2f, %.2f]\n", i, step, posAtStep(i, :));

                end

                for i = 1 : length(boids)
                    if i == obj.ID
                        continue;
                    end

                    % check if other Boids has enter the display cell of
                    % current boid. If so, mark it
                    % if a close boid hasn't been record, and has not
                    % arrived, mark it
%                     if (obj.ID == 28 && i == 27) || (obj.ID == 27 && i == 28)
%                         fprintf("Step %d, Drone %d and %d at [%.4f, %.4f], [%.4f, %.4f], distance: %.4f\n", step, obj.ID, i, posAtStep(obj.ID, :), posAtStep(i, :), norm(posAtStep(obj.ID, :) - posAtStep(i, :)));
%                     end

                    if ~boids(i).arrived && (~any(collisions) || ~ismember(i, collisions(:,1))) && ...
                            obj.dispCellRadius > abs(norm(posAtStep(obj.ID, :) - posAtStep(i, :)))
                        collisions = [collisions; i, step, posAtStep(obj.ID, :)];
                        collidingTimes = collidingTimes + 1;
                    end

                end

                % after check the closest collision, if there are, return
                % the collision information
                if collisions
                    return;
                end

            end

        end


        % get the velocity by multiplying boid's direction with speed
        function velocity = getVelocity(obj)
            velocity = obj.direction * obj.speed;
        end

        
        function potentialPos = getNeighborPos(obj, position)
            potentialPos = [];

            direc = [[0,1];[1,0];[0,-1];[-1,0]];
            for i = 1 : length(direc)
                newPos = position + direc(i, :);
                if all(newPos)
                    potentialPos = [potentialPos; newPos];
                end
            end
        end
    end
end

