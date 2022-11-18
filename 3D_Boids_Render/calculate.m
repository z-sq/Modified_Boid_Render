fileNames = ["pt1619.1727.ptcld", "pt1630.1562.ptcld", "pt1617.1197.ptcld", "pt1620.997.ptcld", "pt1625.760.ptcld", "pt1608.758.ptcld", "pt1609.454.ptcld"];

for i = 1 : length(fileNames)
    pointCloud = convertCellListToMat("./pointclouds/" + fileNames(i));
    minDist  =  inf;
    for x = 1 : size(pointCloud,1)
        for y = x : size(pointCloud,1)
            if x == y
                continue;
            end
            dist = norm(pointCloud(x,:) - pointCloud(y,:));
            minDist = min(minDist, dist);
        end
    end
    disp(minDist);
end