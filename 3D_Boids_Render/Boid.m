classdef Boid
    %BOID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ID = 0;
        maxSpeed = 0;
        maxAcc = 5;
        direction = [0, 0, 0];
        speed = 0;
        acc = 0;
        checkSteps = 0;
        position = [0, 0, 0];
        startPt = [0, 0, 0];
        timeUnit = 0;
        dispCellRadius = 0;
        illumCellRadius = 0;
        avoidAngel = pi/4;
        target = [0, 0, 0];
        arrived = false;
        removed = false;
        distTraveled = 0;
        radioRange = 0;
        goDark = false;
        plan;
        exchangingWith = -1;
    end
    
    methods
        function obj = Boid(ID, initialPosition, maxSpeed, maxAcc, checkSteps, timeunit, dispCellRadius, radioRange, illumCellRadius)
            %BOID Construct an instance of this class
            %   Detailed explanation goes here
            obj.ID = ID;
            obj.maxSpeed = maxSpeed;
            obj.maxAcc = maxAcc;
            obj.speed = obj.maxSpeed;
            obj.checkSteps = checkSteps;
            obj.position = initialPosition;
            obj.timeUnit = timeunit;
            obj.dispCellRadius = dispCellRadius;
            obj.radioRange = radioRange;
            obj.plan = zeros(checkSteps, 6);
            obj.illumCellRadius = illumCellRadius;
        end

        function [obj, avoidingType, positiontype, cantAvoid] = planMove(obj, boids, initial)

            avoidingType = 0;
            positiontype = 0;
            cantAvoid = 0;

            % try to go to target
            [obj, needRePlan] = obj.goToTarget();

            % update direction change to boids
            boids(obj.ID) = obj;


            [obj, collisions] = obj.checkCollision(boids, needRePlan);
            

            % if there are collisions, start avoiding
            if collisions
                [obj, avoidingType, positiontype, cantAvoid] = obj.avoidCollisions(boids, collisions);
            end

            if obj.exchangingWith
                obj.exchangingWith = -1;
            end
        end
    
        function [obj, arrived] = makeMove(obj)
            distMoveInStep = (obj.speed * obj.timeUnit + 0.5 * obj.acc * obj.timeUnit^2);
            
            obj.speed = obj.plan(1,5);
            obj.acc = obj.plan(1,6);
            obj.position = obj.plan(1,1:3);

            obj.distTraveled = obj.distTraveled + distMoveInStep;
            obj.arrived = obj.plan(1,4);
            arrived = obj.arrived;
        end
        
        %   Rule 1: Avoid Collisions
        function [obj, avoidingType, positionChosedType, cantAvoid] = avoidCollisions(obj,boids, collisions)
            avoidingType = 0;
            positionChosedType = 0;
            cantAvoid = 0;
            
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
                    if positionChoose(i,1:3) == obj.position
                        continue;
                    end

                    obj.direction = (positionChoose(i,1:3) - obj.position)/norm((positionChoose(i,1:3) - obj.position));
                    positionChosedType = positionChoose(i,4);
                    boids(obj.ID).direction = obj.direction;

                    [obj, recheckCollisions] =  obj.checkCollision(boids,true);
                    
%                     if i > 50
%                         obj.arrived = true;
%                         cantAvoid = true;
%                         return;
%                     end
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
        function [obj, needRePlan] = goToTarget(obj)
            needRePlan = false;
            towardTarget = (obj.target - obj.position)/norm(obj.target - obj.position);
            if ~(all(abs(obj.direction(:)-towardTarget(:))<0.01))
                obj.direction = towardTarget;
                needRePlan = true;
            end
            if norm(obj.target - obj.position) == 0
                obj.direction = [0,0,0];
            end
        end



        %   This method is used to check collision with other boids
        function [obj,collisions] = checkCollision(obj, boids, needRePlan)

            collisions = [];
            collidingTimes = 0;

            posAtStep = zeros(length(boids), 4, obj.checkSteps);

            % load the position of all boids
            for i = 1 : length(boids)
                posAtStep(i,:,:) = boids(i).plan(:, 1:4).';
%                 fprintf("Drone %d at [%.2f, %.2f]\n", i, posAtStep(i, :));
            end

            % move one step ahead
            obj.plan = [obj.plan(2:end,:); zeros(1,6)];

            for step = 1 : obj.checkSteps
                if ~needRePlan
                    step = obj.checkSteps;
                end
                if step == 1
                    obj.acc = obj.calculateAcc([obj.position,0,obj.speed]);
                    obj.plan(step,1:3) = obj.position + (obj.speed * obj.timeUnit + 0.5 * obj.acc * obj.timeUnit^2)*obj.direction;
                    obj.plan(step, 5) = obj.speed + obj.acc * obj.timeUnit;
                    obj.plan(step, 6) = obj.calculateAcc(obj.plan(step,1:5));
                    obj.plan(step, 4) = 0;
                elseif obj.plan(step - 1, 4)
                    obj.plan(step,:) = obj.plan(step-1,:); 
                else
                    obj.plan(step,1:3) = obj.plan(step - 1,1:3) + (obj.plan(step - 1,5) * obj.timeUnit + 0.5 *  obj.plan(step - 1,6) * obj.timeUnit^2) * obj.direction;
                    obj.plan(step, 5) = obj.plan(step-1, 5) + obj.plan(step-1, 6) * obj.timeUnit;
                    obj.plan(step, 6) = obj.calculateAcc(obj.plan(step,1:5));
                    obj.plan(step, 4) = 0;
                end

                if abs(norm(obj.plan(step,1:3) - obj.target)) < obj.illumCellRadius
                    obj.plan(step, 4) = 1;
                    obj.plan(step, 5:6) = [0,0];
                end
                % update the position
%                  for i = 1 : length(boids)
                 for i = 1 : size(boids,1)
                    if i == obj.ID || i == obj.exchangingWith
                        continue;
                    elseif i > obj.ID
                        if step == obj.checkSteps
                            break;
                        end
                        checkstep = step + 1;
                    else
                        checkstep = step;
                    end


                    % check if other Boids has enter the display cell of
                    % current boid. If so, mark it
                    % if a close boid hasn't been record, and has not
                    % arrived, mark it

                    if ~posAtStep(i,4,step) && ...
                            2 * obj.dispCellRadius > abs(norm(obj.plan(step,1:3) - posAtStep(i, 1:3, checkstep)))
                        collisions = [collisions; i, step, obj.plan(step,1:3)];
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

        % apply the speed model of APF
        function acc = calculateAcc(obj, planOfStep)
            acc = 0;
            distToTarget = abs(norm(planOfStep(1:3) - obj.target));
            distToSlow = 0.5 * ((planOfStep(5)^2)/obj.maxAcc);

            if distToTarget < distToSlow + planOfStep(5) * obj.timeUnit
                acc = max((-planOfStep(5)^2)/(2*distToTarget), -planOfStep(5)/obj.timeUnit);
                acc = max(acc, -obj.maxAcc);
            elseif planOfStep(5) < obj.maxSpeed
                acc = min((obj.maxSpeed - planOfStep(5))/obj.timeUnit, obj.maxAcc);
            end
        end

        
        function markedPos = getNeighborPos(obj, position)
            potentialPos = [];

            direc = [[1,0,0];[-1,0,0];[0,1,0];[0,-1,0];[0,0,1];[0,0,-1]];

            clappingAngles = [];

            
            for i = 1 : length(direc)
                newPos = position + 2 * obj.dispCellRadius * direc(i, :);

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

