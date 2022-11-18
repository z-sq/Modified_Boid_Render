classdef Boid
    %BOID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID = 0;
        maxSpeed = 0;
        direction = [0, 0, 0];
        speed = 0;
        checkSteps = 0;
        position = [0, 0, 0]
        timeUnit = 0;
        dispCellRadius = 0;
        avoidAngel = pi/4;
        target = [0, 0, 0];
        arrived = false;
        removed = false;
        distTraveled = 0;
        radioRange = 0;
        goDark = false;
    end
    
    methods
        function obj = Boid(ID, initialPosition, maxSpeed, checkSteps, timeunit, dispCellRadius, radioRange)
            %BOID Construct an instance of this class
            %   Detailed explanation goes here
            obj.ID = ID;
            obj.maxSpeed = maxSpeed;
            obj.speed = obj.maxSpeed;
            obj.checkSteps = checkSteps;
            obj.position = initialPosition;
            obj.timeUnit = timeunit;
            obj.dispCellRadius = dispCellRadius;
            obj.radioRange = radioRange;
        end

        function [obj, avoidingType, positiontype] = planMove(obj, boids)

            avoidingType = 0;
            positiontype = 0;

            % try to go to target
            obj = obj.goToTarget();

            % update direction change to boids
            boids(obj.ID) = obj;

            % if there are collisions, start avoiding
            collisions = obj.checkCollision(boids);
            if collisions
                [obj, avoidingType, positiontype] = obj.avoidCollisions(boids, collisions);
            end
        end

        function obj = makeMove(obj)
            obj.position = obj.position + obj.getVelocity() * obj.timeUnit;
            obj.distTraveled = obj.distTraveled + obj.speed * obj.timeUnit;
        end
        
        %   Rule 1: Avoid Collisions
        function [obj, avoidingType, positionChosedType] = avoidCollisions(obj,boids, collisions)
            avoidingType = 0;
            positionChosedType = 0;
            
            % Only 2 Boids colliding
            if size(collisions,1) == 1
                avoidingType = 1;
                positionChoose = [];

               % if just 2 Boids colliding
                positionChoose = [positionChoose; obj.getNeighborPos(collisions(3:5))];

                % keep finding until no more collisions
                stillColliding = true;
                i = 0;
                while stillColliding
                    i = i + 1;
                    if ~any(positionChoose)
                        continue;
                    end

                    obj.direction = (positionChoose(i,1:3) - obj.position)/norm((positionChoose(i,1:3) - obj.position));
                    positionChosedType = positionChoose(i,4);
                    boids(obj.ID).direction = obj.direction;

                    recheckCollisions =  obj.checkCollision(boids);

                    % if no collisions, quit; if still collisions, add the
                    % place and
                    if size(recheckCollisions,1) == 0
                        stillColliding = false;
                    elseif size(recheckCollisions,1) >= 1
                        newPosibleChoose = obj.getNeighborPos(positionChoose(i,1:3));
                        positionChoose = [positionChoose; newPosibleChoose];
                        [uniquePos, originIndex, newIndex] = unique(positionChoose(:,1:3), 'rows', "stable");
                        positionChoose = positionChoose(originIndex,:); 
                    end

                end

            elseif size(collisions, 1) > 1

                avoidingType = 2;

                % if multiple Boids Collides, find the one with minimum ID
                % to be the leading boid
                leadingBoidID = min(collisions(:,1));
                
                dotOfDirections = dot(boids(leadingBoidID).direction, obj.direction);
                crossOfDirections = cross(boids(leadingBoidID).direction, obj.direction);

                clappingAngle = atan2(norm(crossOfDirections),dotOfDirections);

                newDirection = (((1-(pi/4)/clappingAngle)) * boids(leadingBoidID).direction + ((pi/4)/clappingAngle) *obj.direction)/2;

                obj.direction = newDirection/norm(newDirection);

%                 transitionMaxis1 = [cos(obj.avoidAngel), sin(obj.avoidAngel);-sin(obj.avoidAngel), cos(obj.avoidAngel)];
%                 transitionMaxis2 = [cos(2*pi - obj.avoidAngel), sin(2*pi - obj.avoidAngel);-sin(2*pi - obj.avoidAngel), cos(2*pi - obj.avoidAngel)];
% 
% 
%                 clappingAngle = atan2d(det([boids(leadingBoidID).direction;obj.direction]),dot(boids(leadingBoidID).direction,obj.direction));
%                 if clappingAngle <= 180
%                     obj.direction = boids(leadingBoidID).direction * transitionMaxis1;
%                 else
%                     obj.direction = boids(leadingBoidID).direction * transitionMaxis2;
%                 end
            end
        end

        % Rule 2: go To Target
        function obj = goToTarget(obj)
            obj.direction = (obj.target - obj.position)/norm(obj.target - obj.position);
            if norm(obj.target - obj.position) == 0
                obj.direction = [0,0,0];
            end
        end



        %   This method is used to check collision with other boids
        function collisions = checkCollision(obj, boids)

            collisions = [];
            collidingTimes = 0;

            posAtStep = zeros(length(boids), 3);
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

                    if ~boids(i).arrived && ...
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

        
        function markedPos = getNeighborPos(obj, position)
            potentialPos = [];

            direc = [[1,0,0];[-1,0,0];[0,1,0];[0,-1,0];[0,0,1];[0,0,-1]];

            clappingAngles = [];

            
            for i = 1 : length(direc)
                newPos = position + direc(i, :);

                newDirection = (newPos - obj.position)/norm(newPos - obj.position);

                clappingAngle = calculateClappingAngle(obj.direction, newDirection);


                if clappingAngle < 0
                    clappingAngle = 2*pi + clappingAngle;
                end

                clappingAngles = [clappingAngles; clappingAngle, i];

                potentialPos = [potentialPos; newPos];
            end

            clappingAngles = sortrows(clappingAngles,1);

            markedPos = [];

            for i = 1: length(clappingAngles)
                if all(potentialPos(clappingAngles(i,2),:))
                    % i value means the index of clapping angle, where 1 is
                    % the position ahead, 6 is the behiend, 2-5 are sides
                    markedPos = [markedPos; potentialPos(clappingAngles(i,2),:), i];
                end
            end

        end
    end
end

