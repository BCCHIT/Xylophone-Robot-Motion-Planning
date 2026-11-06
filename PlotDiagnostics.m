function PlotDiagnostics(t, Cart_Traj, q, qd, qdd, waypointsBase, eventTable, cfg)
% PlotDiagnostics
% 绘制课程要求中最关键的诊断图：笛卡尔轨迹、关节角、一阶差分/速度、二阶差分/加速度。

if ~exist(cfg.outputDir, 'dir')
    mkdir(cfg.outputDir);
end

figure('Name','Cartesian trajectory');
plot3(Cart_Traj(:,1), Cart_Traj(:,2), Cart_Traj(:,3), 'LineWidth', 1.2);
hold on;
scatter3(waypointsBase(:,1), waypointsBase(:,2), waypointsBase(:,3), 25, 'filled');
grid on; axis equal;
xlabel('X / mm'); ylabel('Y / mm'); zlabel('Z / mm');
title('木琴演奏笛卡尔轨迹');
saveas(gcf, fullfile(cfg.outputDir, 'fig_cartesian_trajectory.png'));

figure('Name','Joint angle');
plot(t, rad2deg(q), 'LineWidth', 1.0);
grid on;
xlabel('t / s'); ylabel('q / deg');
title('关节角轨迹');
legend('q1','q2','q3','q4','q5','q6', 'Location','best');
saveas(gcf, fullfile(cfg.outputDir, 'fig_joint_angle.png'));

figure('Name','Joint velocity');
plot(t, qd, 'LineWidth', 1.0);
grid on;
xlabel('t / s'); ylabel('dq/dt / rad/s');
title('关节一阶差分/速度诊断');
legend('q1','q2','q3','q4','q5','q6', 'Location','best');
saveas(gcf, fullfile(cfg.outputDir, 'fig_joint_velocity.png'));

figure('Name','Joint acceleration');
plot(t, qdd, 'LineWidth', 1.0);
grid on;
xlabel('t / s'); ylabel('d2q/dt2 / rad/s^2');
title('关节二阶差分/加速度诊断');
legend('q1','q2','q3','q4','q5','q6', 'Location','best');
saveas(gcf, fullfile(cfg.outputDir, 'fig_joint_acceleration.png'));

% 保存事件表，方便查找每个音符的 safe/hit/lift 时间。
try
    writetable(eventTable, fullfile(cfg.outputDir, 'event_table.csv'));
catch
    % 老版本 MATLAB 没有 writetable 时忽略。
end
end
