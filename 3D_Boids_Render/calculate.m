fileNames = ["./Point Cloud Squence/pt1605_change.ptcld","./Point Cloud Squence/pt1709_change.ptcld","./Point Cloud Squence/pt1811_change.ptcld","pt1547_change.ptcld", "./Point Cloud Squence/pt1379_change.ptcld"];

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