%% VerifyKinematics.m
% 正逆运动学快速检查脚本。

clear; clc;
cfg = RobotConfig_HL0905();

theta_test = [pi/3, pi/4, pi/2, pi/6, pi/3, pi/6];
T_calc = Fkine(cfg.Xi, theta_test, cfg.g0);

fprintf('正运动学 T_calc = \n');
disp(T_calc);

theta_sols_all = Ikine6s_all(T_calc, cfg.config);
fprintf('全解数量：%d\n', size(theta_sols_all, 2));
disp(theta_sols_all.');

theta_one = Ikine6s(T_calc, cfg.config);
fprintf('单组逆解：\n');
disp(theta_one);
