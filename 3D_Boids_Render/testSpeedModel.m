clc; close all;

startPoint = [0, -3];

position = [0, 0];

speed = 3;

maxSpeed = 3;
maxAcc = 3;

direction = [0, 1];

target = [0, 10];

timeunit = 1/25;

step = 0;

C = 25;

plot(0,0);
hold on;

while norm(position - target) > 0.2
    step = step + 1;
    l = min(norm(position - startPoint), norm(position - target))/25;
% 
%     l = min(l, speed * timeunit + 0.5 * maxAcc * timeunit^2);
%     l = max(l, speed * timeunit - 0.5 * maxAcc * timeunit^2);


    l = min(l, maxSpeed * timeunit);
%     l = max(l, 0.5 * maxAcc * timeunit^2);

    newspeed = l * 2/timeunit - speed;
    acc = (newspeed - speed)/timeunit;
    speed = newspeed;

    position = position + l * direction;
    
    fprintf("Step %d , position [%.4f, %.4f], l %.4f, speed %.4f, acc %.4f\n", step, position, l, speed, abs(acc));

    plot(position(1),position(2), '.');
    hold on;
end