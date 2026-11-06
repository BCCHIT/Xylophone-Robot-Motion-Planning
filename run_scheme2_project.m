%% run_scheme2_project.m
% 方案2：使用 15 键实测敲击关节角，并在关节空间做 LFPB。

clear; clc; close all;

cfg = RobotConfig_HL0905();
cfg.outputDir = fullfile(pwd, 'output');

cfg.scheme2.approachHeight = 12;
cfg.scheme2.reboundHeight = 10;
cfg.scheme2.strikeDepth = 1;
cfg.scheme2.downRatio = 0.16;
cfg.scheme2.upRatio = 0.16;
cfg.scheme2.finalReturnTime = 2.0;

keyMap = Scheme2KeyMap();
cfg.scheme2.qSeed = keyMap.qHit(1,:);

[notes, beats, BPM] = Song_Scheme2Placeholder();

[qWaypoints, waypointTimes, cartWaypoints, eventTable] = BuildScheme2JointWaypoints(notes, beats, BPM, keyMap, cfg);
[q, qdRef, qddRef, t] = SampleTimedLFPBJointTrajectory(qWaypoints, waypointTimes, cfg.Ts, cfg.blendRatio);
q = unwrap(q, [], 1);

[qd, qdd] = NumericalDiff(t, q);
qd = smoothdata(qd, 'movmean', cfg.smoothWindow);
qdd = smoothdata(qdd, 'movmean', cfg.smoothWindow);

cartFromFk = zeros(size(q,1), 3);
for i = 1:size(q,1)
    Ti = Fkine(cfg.Xi, q(i,:), cfg.g0);
    cartFromFk(i,:) = Ti(1:3,4).';
end

ExportScheme2PointFiles(t, q, cartFromFk, cfg);
PlotDiagnostics(t, cartFromFk, q, qd, qdd, cartWaypoints, eventTable, cfg);

assignin('base','t',t);
assignin('base','q',q);
assignin('base','qd',qd);
assignin('base','qdd',qdd);
assignin('base','scheme2_qWaypoints',qWaypoints);
assignin('base','scheme2_cartFromFk',cartFromFk);
assignin('base','scheme2_eventTable',eventTable);

fprintf('\n方案2运行完成。\n');
fprintf('已生成：output/joint.txt\n');
fprintf('当前乐谱为《天空之城》简化片段；修改 Song_Scheme2Placeholder.m 可更换音符与节拍。\n');
