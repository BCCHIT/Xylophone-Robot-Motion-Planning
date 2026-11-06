function ExportPointFiles(t, Cart_Traj, q, R_target, cfg)
% ExportPointFiles
% 导出课程设计需要的点位文件。
% cart_pos.txt: x y z rx ry rz，位置单位 mm，姿态单位 deg。
% joint_pos.txt: q1~q6，单位 deg。
% joint_pos_downsampled.txt: 降采样后的关节点位，便于真机测试。

if ~exist(cfg.outputDir, 'dir')
    mkdir(cfg.outputDir);
end

cartFile = fullfile(cfg.outputDir, 'cart.txt');
jointFile = fullfile(cfg.outputDir, 'joint.txt');
jointFileDs = fullfile(cfg.outputDir, 'joint_downsampled.txt');
timeFile = fullfile(cfg.outputDir, 'time.txt');

% 笛卡尔点位。这里沿用固定姿态记录；若控制器要求不同欧拉角顺序，应在此处修改。
fid = fopen(cartFile, 'w');
for i = 1:cfg.exportEvery:size(Cart_Traj,1)
    fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f\n', ...
        Cart_Traj(i,1), Cart_Traj(i,2), Cart_Traj(i,3), ...
        cfg.eulerDegForCartFile(1), cfg.eulerDegForCartFile(2), cfg.eulerDegForCartFile(3));
end
fclose(fid);

% 关节点位，单位 degree。
fid = fopen(jointFile, 'w');
for i = 1:cfg.exportEvery:size(q,1)
    qDeg = rad2deg(q(i,:));
    fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f\n', qDeg);
end
fclose(fid);

fid = fopen(jointFileDs, 'w');
for i = 1:cfg.exportEveryDownsampled:size(q,1)
    qDeg = rad2deg(q(i,:));
    fprintf(fid, '%.6f %.6f %.6f %.6f %.6f %.6f\n', qDeg);
end
fclose(fid);

fid = fopen(timeFile, 'w');
for i = 1:numel(t)
    fprintf(fid, '%.6f\n', t(i));
end
fclose(fid);
end
