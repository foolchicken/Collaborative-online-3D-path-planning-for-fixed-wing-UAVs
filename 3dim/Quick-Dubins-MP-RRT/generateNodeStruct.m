StructNode.pos = []; % 当前节点的位置

StructNode.state = []; % 当前节点的状态量（xyh、yaw、pitch）
StructNode.path = []; % 上一节点到当前接节点的转移路径
StructNode.hterrain = inf; % 当前节点位置的地形高度

StructNode.h = inf; % 当前节点的启发值
StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.dh = []; % 衡量高度的起伏
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引
StructNode.obs = []; % 受影响的障碍物
%StructNode.child = []; % 子节点在树中的索引

StructNode.ifact = true; % 是否参与计算