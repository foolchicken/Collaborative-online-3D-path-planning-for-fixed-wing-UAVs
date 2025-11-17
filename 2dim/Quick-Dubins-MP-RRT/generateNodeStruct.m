
StructNode.theta0 = []; % 该连接的起始航向角
StructNode.thetaf = [];
StructNode.path = []; % 上一节点到当前接节点的转移路径

StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引
StructNode.h = inf; % 当前节点的启发值
StructNode.ifact = true; % 是否参与计算
StructNode.pos = []; % 当前节点位置