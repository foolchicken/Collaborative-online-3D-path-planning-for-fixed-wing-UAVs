function isvalid = PathCollisionCheck(path, algoParam)
% 路径碰撞检查，true为可行路径

isvalid = false;
for i = 1 : algoParam.checkStep : size(path, 1)
    if ~PointCheck(path(i,:), algoParam)
        return
    end
end
isvalid = true;


end

