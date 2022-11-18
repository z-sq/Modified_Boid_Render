function pointCloud= convertCellListToMat(filename)
    vertexList = readPrincetonFile(filename, 1);

    pointCloud = zeros(size(vertexList,2),3);
    for i = 1 : size(vertexList,2)
        pointCloud(i,:) = vertexList{i};
    end
end

