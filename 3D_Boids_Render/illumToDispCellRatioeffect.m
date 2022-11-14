
ratios = [1,1.25,2.5,3,5,8,10,13,15,20,25,30];
result = [];

for i = 1: length(ratios)
    [totalCollisions, exchangeTriggered, twoColliding, multipleColliding, stepsForPtCld] = renderPointClouds(ratios(i), i);
    result = [result; ratios(i), totalCollisions, exchangeTriggered, twoColliding, multipleColliding, stepsForPtCld];
end


writematrix(result, "ratioEffects_Rule2_changed_2.xlsx");

