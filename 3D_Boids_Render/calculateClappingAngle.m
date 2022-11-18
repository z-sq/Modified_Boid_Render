function clappingAngle = calculateClappingAngle(baseVector, calculateVector)
    dotOfDirections = dot(baseVector, calculateVector);
    crossOfDirections = cross(baseVector, calculateVector);
    clappingAngle = atan2(norm(crossOfDirections),dotOfDirections);
end

