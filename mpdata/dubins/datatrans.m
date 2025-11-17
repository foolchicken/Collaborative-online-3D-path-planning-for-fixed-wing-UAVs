clear
clc

matFiles = dir(fullfile('D:\Matlabproject\硕士毕业论文\mpdata\dubins', '*.mat'));

% 依次读取每个.mat文件
for k = 1 : length(matFiles)
    % 获取文件名
    fileName = matFiles(k).name;

    % 加载.mat文件
    data = load(fileName);

    % 处理数据
    ind1 = find(fileName == 'v');
    ind2 = find(fileName == '.');
    speed = str2num(fileName(ind1 + 1 : ind2 - 1));
    fieldNames = fieldnames(data);
    database = data.(fieldNames{1});

    thetadisNum = 24;
    gridres = database{end, end}(1).path(end, 1) / (size(database, 1) - 1);

    if contains(fileName,'FTA')
        mpdataFTA.database = database;
        mpdataFTA.thetadisNum = thetadisNum;
        mpdataFTA.speed = speed;
        mpdataFTA.gridres = gridres;
        save(['mpdataFTAv' num2str(speed) '.mat'], 'mpdataFTA');
    else
        mpdata.database = database;
        mpdata.thetadisNum = thetadisNum;
        mpdata.speed = speed;
        mpdata.gridres = gridres;
        save(['mpdatav' num2str(speed) '.mat'], 'mpdata');
    end

end

