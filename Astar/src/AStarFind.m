%function AStarFind()
    obstlist = [0,0;1 3;2 2;5,5]; % 障碍物中心位置，假设单位是米
    gres = 0.2; % 栅格尺寸为0.2米
    [minx,miny,obmap] = CalcObstMap(obstlist,gres); % minx miny其实是地图的
    
    % 后缀带i代表是栅格坐标，不带i代表是实际坐标 
    goal = [2.5,3.5];
    start_i = [4,3];
    col = goal(1); % 实际坐标与栅格坐标转换，x坐标变成列数，
    row = goal(2); % y坐标变成行数
    col = ceil((col-minx)/gres); % 将goal转换为栅格坐标，四舍五入为整数
    row = ceil((row-miny)/gres);
    goal_i = [row,col];
    start = (start_i - 0.5)*gres+[minx,miny]; % 将start_i栅格坐标转换为实际坐标
    
    [Grids, cost] = AStarSearch(start_i,goal_i,obmap); % 核心搜索算法
    
    % 计算最优路径
    pointIndex = goal_i;
    pathPoints = []; % 存放最优路径
    while any(pointIndex~=start_i) % 回溯到起始栅格点时，找到最优路径
        pathPoints(end+1,:) = (pointIndex-0.5)*gres+[minx,miny]; % 把栅格距离转换为实际距离，这样的写法类似vector的push_back,也可以写成pathPoints=[pathPoints;xxxxx],注意需前面必须要出现这个变量才能这么使用
        xi = Grids(pointIndex(1),pointIndex(2),1); % pointIndex的父节点的行坐标
        yi = Grids(pointIndex(1),pointIndex(2),2); % pointIndex的父节点的列坐标
        pointIndex = [xi, yi]; % 原来pointIndex的父节点栅格坐标
    end
    pathPoints(end+1,:) = (pointIndex-0.5)*gres+[minx,miny]; % 把起始栅格点加入路径中
    
    % 画图
    draw(minx,miny,gres,obmap,start,goal,obstlist,pathPoints);
    disp(['cost is ',num2str(cost)]);
%end
function draw(minx,miny,gres,obmap,start,goal,obstlist,pathPoints)
    hold on;
    grid on
    xlabel('x/m');
    ylabel('y/m');
    set(gca,'xaxislocation','top','yaxislocation','left','ydir','reverse') % % 以左上角为原点建立坐标系
    % 绘制障碍栅格地图，实际坐标不用修改，栅格坐标需要反转下
    for i = 1:size(obmap,1) % 注意在画图的时候，栅格地图中的行数i代表实际地图中的y，所以这里索引要对应修改
        for j = 1:size(obmap,2)
            if obmap(i,j) == 1
                rectangle('Position',[[minx,miny]+([j,i]-1)*gres,gres,gres],'FaceColor',[0 .1 .1]) % rectangle('Position',pos)，os 指定为 [x y w h] 形式的四元素向量（以数据单位表示）.x 和 y 元素确定位置，w 和 h 元素确定栅格的宽度和高度.
            else
                rectangle('Position',[[minx,miny]+([j,i]-1)*gres,gres,gres],'FaceColor',[1  1  1]) % 绘制的时候是从0开始绘制的
            end
        end
    end
    % 注意在画图的时候，栅格地图中的行数i代表实际地图中的y，所以这里索引要对应修改    
    plot(goal(1),goal(2),'ro'); % goal是实际坐标不用修改
    plot(start(2),start(1),'*'); % start是栅格地图坐标，栅格地图中的行数i代表实际地图中的y
    for i = 1:size(obstlist,1)
        plot(obstlist(i,1),obstlist(i,2),'bo');
    end

    plot(pathPoints(:,2),pathPoints(:,1)); 
end
function [minx,miny,obmap] = CalcObstMap(obstlist,gres) % 使用障碍物中点的最大最小坐标构建地图
    minx = min(obstlist(:,1));
    maxx = max(obstlist(:,1));
    miny = min(obstlist(:,2));
    maxy = max(obstlist(:,2));
    xwidth = maxx - minx; % 计算距离
    xwidth = ceil(xwidth/gres); % 计算栅格个数，四舍五入到整数
    ywidth = maxy - miny;
    ywidth = ceil(ywidth/gres);
    obmap = zeros(ywidth,xwidth); % 初始化栅格地图，值全为0
    for i = 1:ywidth
        for j = 1:xwidth
            ix = minx+(j-1/2)*gres; % 把离散栅格转换为实际位置坐标，单位是米
            iy = miny+(i-1/2)*gres;
            [~,D] = knnsearch(obstlist,[ix,iy]); %计算点(ix, iy)距离obstlist中点最近的一个点的距离
            if D < 0.5 % 距离小于0.5则认为是障碍物的一部分，障碍物的边长，形状为正方形
                obmap(i,j) = 1;
            end
        end
    end
end
function [Grids, cost] = AStarSearch(start,goal,obmap)
    dim = size(obmap); % 计算栅格地图尺寸，返回m*n，m行，n列
    Grids = zeros(dim(1),dim(2),4); 
    for i = 1:dim(1)
        for j = 1:dim(2)
            Grids(i,j,1) = i; % 栅格地图中点(i,j)的父节点的栅格行坐标
            Grids(i,j,2) = j; % 栅格地图中点(i,j)的父节点的栅格列坐标
            Grids(i,j,3) = norm(([i,j]-goal)); % 构造栅格的启发值h
            Grids(i,j,4) = inf; % 从起始点到当前点i,j走过的距离，实际就是g值
        end
    end
    Open = start; % Open集合，存放待拓展的栅格节点，不是实际坐标
    Grids(start(1),start(2),4) = 0; % 从起始点到该点的距离
    Close = []; % Close集合，存放拓展过的栅格节点，不是实际坐标
    while ~isempty(Open)% 当open为空时，搜索完毕，为A*核心搜索代码的三个过程1.找出最小的点 2.拓展点 3.把点放入close集合中
        [wknode,Open] = PopOpen(Open,Grids); % 找出估计值f最小的点wknode，f=g+h，然后从Open中删除该点
        [Grids,Open,Close,target_flag] = Update(wknode,goal,obmap,Grids,Open,Close); % 拓展该点wknode
        Close(end+1,:) = wknode; %#ok<AGROW> 把该点wknode放入close集合中
        if target_flag % 搜索成功
            break
        end
    end
    cost = Grids(goal(1),goal(2),3) + Grids(goal(1),goal(2),4); % f=g+h，goal处的启发值为0，所以Grids(goal(1),goal(2),3)=0,可能存在找不到路径的情况
end

function [Grids,Open,Close,target_flag] = Update(wknode,goal,obmap,Grids,Open,Close)
    dim = size(obmap); % 行和列数目：m*n
    target_flag=0; % 搜索到最优路径了
    for i = -1:1 % i,j代表8种运动方式，上下左右，左上，右上，左下，右下
        for j = -1:1
            adjnode = wknode+[i,j]; % 根据不同运动方式计算邻接栅格点的坐标
            row = adjnode(1);
            col = adjnode(2);
            if i == 0 && j == 0 % 忽略自身
                continue
            elseif row < 1 || row > dim(1) % 忽略行数组越界
                continue
            elseif col < 1 || col > dim(2) % 忽略列数组越界
                continue
            elseif obmap(row,col) == 1 % 忽略是障碍物的栅格
                continue
            elseif ~isempty(Close) && ismember(adjnode,Close,'rows') % 忽略是Close集合中的栅格
                continue
            end
            fcost = Grids(wknode(1),wknode(2),4)+norm([i,j]); %计算当前点到邻接栅格点的f值，f=g+h
            if Grids(row,col,4) > fcost % 邻接栅格点的旧f值比新f值大，更新f值
                Grids(row,col,1) = wknode(1); % 更新邻接栅格点的父节点行坐标
                Grids(row,col,2) = wknode(2); % 更新邻接栅格点的父节点列坐标
                Grids(row,col,4) = fcost; % 更新邻栅格点的f值
                % 如果邻接栅格点不存在并且栅格点也不是目标点
                if ~ismember(adjnode,Open,'rows')
                    if ~isequal(adjnode,goal)
                        Open(end+1,:) = adjnode; %#ok<AGROW
                    else
                        Open(end+1,:) = adjnode; %#ok<AGROW
                        target_flag=1;
                        return
                    end
                end
            end
        end
    end
end

function [wknode,Open] = PopOpen(Open,Grids)
    mincost = inf; % 记录最小的f值
    minidx = 1; % 最小f值的栅格点索引
    for i = 1:size(Open,1) % 找出最小f值的栅格点，size(Open,1)是Open集合中点的数目，Open是一个多行，2列的矩阵
        node = Open(i,:); % 取出第i个栅格节点的栅格坐标
        fcost = Grids(node(1),node(2),3)+Grids(node(1),node(2),4); % 计算当前栅格点的估计值f=g+h
        if fcost < mincost % 找最小f值的栅格节点
            minidx = i;
            mincost = fcost;
        end
    end
    wknode = Open(minidx,:); %取出坐标
    Open(minidx,:) = []; % 删除该栅格点，后面的栅格点前移
end
