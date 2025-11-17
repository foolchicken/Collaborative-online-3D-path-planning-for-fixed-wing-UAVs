
StructNode.pose = []; % 当前节点的位姿
StructNode.path = []; % 父节点到当前节点的dubins路径
StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引