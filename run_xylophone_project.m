%% run_xylophone_project.m
% 木琴演奏课程设计主程序
% 作用：
% 1. 读取乐谱 notes / beats；
% 2. 根据实测木琴琴键坐标生成“准备点-敲击点-回弹点”；
% 3. 用固定时长 LFPB 生成笛卡尔轨迹；
% 4. 用全解逆运动学 + 最小关节跳变原则生成 q, qd, qdd；
% 5. 导出点位文件，并绘制差分诊断图。
%
% 注意：
% keyCentersBase 中的坐标必须在上机后替换为真实测量值。
% 如果控制器已经把 TCP 设置到塑料锤头，则把 cfg.hammerOffsetTool = [0;0;0]。

clear; clc; close all;

cfg = RobotConfig_HL0905();

% 课程要求至少演奏《小星星》
[notes, beats, BPM] = Song_Twinkle();

% 木琴坐标：当前是可仿真的占位值；上机必须替换为实测值
keyMap = XylophoneKeyMap();

% 由乐谱生成带绝对时间戳的关键点
[waypointsBase, waypointTimes, eventTable] = BuildXylophoneWaypoints(notes, beats, BPM, keyMap, cfg);

% 按全局时间轴采样。这里不是每段从 t=0 重新采样，因此能避免“每段时间不是 Ts 整数倍”造成的累积误差。
[Cart_Traj, Cart_Vel, Cart_Acc, t] = SampleTimedLFPBTrajectory(waypointsBase, waypointTimes, cfg.Ts, cfg.blendRatio);

% 固定末端姿态：先采用课设 PPT 中推荐的“垂直敲击方案”
% R_target 使工具 Z 轴朝向基坐标 -Z，并使塑料锤杆方向基本保持水平。
R_target = cfg.R_verticalHit;

% 逆运动学：对每个笛卡尔点求全解，再选与上一时刻关节角最接近的一组，避免关节角跳变。
[q, ikInfo] = SolveIKTrajectory(Cart_Traj, R_target, cfg);

% 关键：消除 ±pi 边界造成的关节角换支跳变
q = unwrap(q, [], 1);

% 根据非均匀时间轴计算 qd/qdd。通常 t 基本等间隔，但最后一个点可能为了精确结束时间被追加。
[qd, qdd] = NumericalDiff(t, q);
qd  = smoothdata(qd,  'movmean', cfg.smoothWindow);
qdd = smoothdata(qdd, 'movmean', cfg.smoothWindow);

% 导出文件：cart_pos.txt / joint_pos.txt / joint_pos_downsampled.txt
ExportPointFiles(t, Cart_Traj, q, R_target, cfg);

% 诊断图：轨迹、关节角、一阶差分、二阶差分
PlotDiagnostics(t, Cart_Traj, q, qd, qdd, waypointsBase, eventTable, cfg);

% 供 Simulink From Workspace 使用。HL0905_2.slx 通常读取 t, q, qd, qdd。
assignin('base','t',t);
assignin('base','q',q);
assignin('base','qd',qd);
assignin('base','qdd',qdd);
assignin('base','Cart_Traj',Cart_Traj);
assignin('base','R_target',R_target);
assignin('base','ikInfo',ikInfo);

fprintf('\n运行完成。\n');
fprintf('已生成：output/cart_pos.txt, output/joint_pos.txt, output/joint_pos_downsampled.txt\n');
fprintf('若要仿真，请把 HL0905_2.slx 放在当前项目根目录，然后打开模型运行。\n');

% 如需自动打开模型，取消下面三行注释：
% if exist('HL0905_2.slx','file')
%     open_system('HL0905_2.slx');
% end
