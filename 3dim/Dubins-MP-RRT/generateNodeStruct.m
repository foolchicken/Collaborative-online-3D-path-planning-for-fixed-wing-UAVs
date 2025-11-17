StructNode.yaw0 = []; % 该连接的起始航向角
StructNode.yawf = [];
StructNode.pitch0 = [];
StructNode.pitchf = [];
StructNode.pos = []; % 当前节点的位置
StructNode.path = []; % 上一节点到当前接节点的转移路径

StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引
