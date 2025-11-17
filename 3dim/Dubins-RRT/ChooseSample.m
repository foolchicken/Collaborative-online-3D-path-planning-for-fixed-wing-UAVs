function sample = ChooseSample(algoParam)
% 选择采样点，输入算法参数

if rand < 0.7 % 偏向目标的RRT
    while 1
        pos = [rand() * (algoParam.mapRef.LongitudeLimits(2) - algoParam.mapRef.LongitudeLimits(1)) + algoParam.mapRef.LongitudeLimits(1), ...
            rand() * (algoParam.mapRef.LatitudeLimits(2) - algoParam.mapRef.LatitudeLimits(1)) + algoParam.mapRef.LatitudeLimits(1)];
        [I, J] = geographicToDiscrete(algoParam.mapRef, pos(2), pos(1));
        if ~isnan(I) && ~isnan(J)
            break
        end
    end
    ht = algoParam.highData(I, J);
%     sample = [pos, rand() *6000 + 3000, ...
%         rand() * 2 * pi, rand() * (algoParam.dynamicCons.pitchmax - algoParam.dynamicCons.pitchmin) + algoParam.dynamicCons.pitchmin];
    sample = [pos, rand() * (algoParam.dynamicCons.maxh - ht) + ht, ...
        rand() * 2 * pi, rand() * (algoParam.dynamicCons.pitchmax - algoParam.dynamicCons.pitchmin) + algoParam.dynamicCons.pitchmin];
    sample(3) = 3000;
else
    sample = algoParam.goal; % 向终点前进
end

originPoint = algoParam.start(1 : 3); % 以起点作为发射系原点
[sample(1), sample(2), sample(3)] = geodetic2enu(sample(2), sample(1), sample(3), originPoint(2), originPoint(1), originPoint(3), wgs84Ellipsoid);

end
