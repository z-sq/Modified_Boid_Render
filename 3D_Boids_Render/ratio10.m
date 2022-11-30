
% ratios = [1,1.25,2.5,3,5,8,10,13,15,20,25,30];
ratios = [2.5, 1, 8 ,10];
result = [];

% for i = 1: length(ratios)
i = 4;
[totalCollisions, exchangeTriggered, twoColliding, multipleColliding, stepsForPtCld] = renderPointClouds(ratios(i), i);
result = [result; ratios(i), totalCollisions, exchangeTriggered, twoColliding, multipleColliding, stepsForPtCld];

writematrix(result, "ratioEffects.xlsx", 'Sheet', i);
% end



