function angle = wrapTo2Pimy(angle)
% 重载matlab的wrapTo2Pi函数，自带的太繁琐，限制在正负pi之间

if angle < -pi
    angle = angle + 2 * pi;
elseif angle > pi
    angle = angle - 2 * pi;
end

end
