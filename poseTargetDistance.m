function [pDist, qDist] = poseTargetDistance(q,target)

    pose = FK_fn(q);

    pDist = norm(target(1:3)-pose(1:3));
    qDist = 1-dot(target(4:7),pose(4:7))^2;

end