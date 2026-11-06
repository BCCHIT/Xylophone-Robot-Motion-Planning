function cfg = RobotConfig_HL0905()
% RobotConfig_HL0905
% 统一放置机器人、塑料锤、轨迹规划、导出文件等参数。

% ---------------- 机器人几何参数，沿用实验三/实验四 ----------------
cfg.L1 = 491;
cfg.L2 = 450;
cfg.L3 = 450;
cfg.L4 = 84;
cfg.config = [cfg.L1, cfg.L2, cfg.L3, cfg.L4];

cfg.g0 = [ -1,  0,  0,    0;
            0, -1,  0,    0;
            0,  0,  1, 1475;
            0,  0,  0,    1];

cfg.Xi = [ 0,   -cfg.L1,  -(cfg.L1+cfg.L2),  0,  -(cfg.L1+cfg.L2+cfg.L3),  0;
           0,    0,       0,                 0,   0,                      0;
           0,    0,       0,                 0,   0,                      0;
           0,    0,       0,                 0,   0,                      0;
           0,    1,       1,                 0,   1,                      0;
           1,    0,       0,                 1,   0,                      1];

% 固定垂直敲击姿态。实验四已经验证过“末端向下”的姿态选择思路。
% R_g0 = cfg.g0(1:3,1:3);
% R_down = [cos(pi) 0 sin(pi); 0 1 0; -sin(pi) 0 cos(pi)];
% cfg.R_verticalHit = R_g0 * R_down;

q_measured_deg = [-15.183,24.339,117.143,0.884,41.982,168.324];   % 测 p1 时示教器显示的六个关节角
T_measured = Fkine(cfg.Xi, deg2rad(q_measured_deg), cfg.g0);
cfg.R_verticalHit = T_measured(1:3,1:3);

% cart_pos.txt 仍记录示教器显示的欧拉角；逆解姿态只使用上面的 Fkine 结果，
% 避免因控制器欧拉角顺序不明导致姿态矩阵不一致。

% ---------------- 塑料锤参数 ----------------
% 塑料锤平面图给出的锤头到安装位置距离约 177.54 mm。
% 这里假设锤头中心在工具坐标系 +X 方向。
% 若在真实控制器中已经把 TCP 设置到锤头中心，则把该项改成 [0;0;0]。
cfg.hammerOffsetTool = [0; 0; 0];

% ---------------- 敲击关键点参数 ----------------
cfg.approachHeight = 15;      % 准备点：琴键表面上方 mm
cfg.reboundHeight  = 12;      % 回弹点：琴键表面上方 mm
cfg.strikeDepth    = 0.8;       % 命令敲击点：略低于琴键表面 mm，用于保证接触时有速度
cfg.downRatio      = 0.12;    % 单个音符时长中，用于下压的比例
cfg.upRatio        = 0.12;    % 单个音符时长中，用于回弹的比例
cfg.finalReturnTime = 2.0;    % 演奏结束后回到安全点的时间，单位 s；不需要可设 0

% ---------------- 轨迹采样与平滑 ----------------
cfg.Ts = 0.001;               % PPB 点位文件没有时间列，按实验四做法使用 1 ms/行。
cfg.blendRatio = 0.20;        % LFPB 加减速段占单段时间比例，0~0.5
cfg.smoothWindow = 9;         % qd/qdd 数值微分后的滑动平均窗口

% ---------------- 逆解选择参数 ----------------
cfg.qSeed = deg2rad([-15.183, 24.339, 117.143, 0.884, 41.982, 168.324]);  % 第一帧的选解参考值，需要上机后按实际初始姿态调整
cfg.jointWeights = [1.5, 1.2, 1.2, 0.8, 1.0, 1.5];
cfg.singularityPenaltyWeight = 0.02;

% 关节限位。这里用宽松值防止误删；上机后应改成机器人实际软限位。
cfg.jointLower = deg2rad([-180, -140, -160, -180, -140, -360]);
cfg.jointUpper = deg2rad([ 180,  140,  160,  180,  140,  360]);

% ---------------- 导出设置 ----------------
cfg.outputDir = fullfile(pwd, 'output');
cfg.exportEvery = 1;          % joint_pos.txt 每隔几个采样点导出一次
cfg.exportEveryDownsampled = 10; % 仅供快速查看/仿真抽样，不建议直接用于 PPB 上机
cfg.eulerDegForCartFile = [155.115, 176.489, 159.298]; % 与 R_verticalHit 对应的简单姿态记录；真实控制器如有固定格式再调整
end
