function isvalid = PathCollisionCheck(path, algoParam)
% 检测路径是否和地形发生碰撞

isvalid = true;
for i = 1 : size(path, 1)
    if ~PointCheck(path(i, :), algoParam)
        isvalid = false;
        return
    end
end



end



