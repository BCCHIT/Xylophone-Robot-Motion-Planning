function theta_sols = Ikine6s_all(T, config)
% Ikine6s_all
% 基于旋量法 / Paden-Kahan 思路的六轴机器人逆运动学全解函数。
% 输出 theta_sols 为 6 x N，每一列是一组解，单位 rad。

L1 = config(1); L2 = config(2); L3 = config(3); %#ok<NASGU>

g0 = [ -1,  0,  0,    0;
        0, -1,  0,    0;
        0,  0,  1, 1475;
        0,  0,  0,    1];

Xi = [ 0,   -L1,  -(L1+L2),  0,  -(L1+L2+L3),  0; 
       0,    0,     0,       0,    0,          0;  
       0,    0,     0,       0,    0,          0;  
       0,    0,     0,       0,    0,          0;  
       0,    1,     1,       0,    1,          0;  
       1,    0,     0,       1,    0,          1]; 

g1 = T / g0; 
rw = [0; 0; L1+L2+L3; 1];
p_target = g1 * rw; 
px = p_target(1);
py = p_target(2);

theta_sols = [];

th1_1 = atan2(py, px); 
th1_2 = th1_1 + pi;
if th1_2 > pi
    th1_2 = th1_2 - 2*pi;
end
theta1_opts = [th1_1, th1_1, th1_2, th1_2];

for i = 1:4
    th1 = theta1_opts(i);

    exp_minus_xi1 = twist_exp(-Xi(:,1), th1);
    p_prime = exp_minus_xi1 * p_target; 

    r = p_prime(1); 
    s = p_prime(3) - L1;

    D = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
    if abs(D) > 1
        continue;
    end

    if mod(i, 2) == 1
        th3 = atan2(-sqrt(1 - D^2), D); 
    else
        th3 = atan2(sqrt(1 - D^2), D); 
    end

    th2 = atan2(r, s) - atan2(L3 * sin(th3), L2 + L3 * cos(th3));

    g123 = twist_exp(Xi(:,1), th1) * twist_exp(Xi(:,2), th2) * twist_exp(Xi(:,3), th3);
    g456 = g123 \ g1; 
    R456 = g456(1:3, 1:3);

    for flip = [1, -1]
        th5 = atan2(flip * sqrt(R456(1,3)^2 + R456(2,3)^2), R456(3,3)); 

        if abs(sin(th5)) > 1e-4
            if flip == 1
                th4 = atan2(R456(2,3), R456(1,3));
                th6 = atan2(R456(3,2), -R456(3,1));
            else
                th4 = atan2(-R456(2,3), -R456(1,3));
                th6 = atan2(-R456(3,2), R456(3,1));
            end
        else
            th4 = 0; 
            if R456(3,3) > 0
                th6 = atan2(R456(2,1), R456(1,1));
            else
                th6 = -atan2(R456(2,1), R456(1,1));
            end
        end

        sol = [th1; th2; th3; th4; th5; th6];
        theta_sols = [theta_sols, sol]; %#ok<AGROW>
    end
end
end

function T = twist_exp(xi, theta)
v = xi(1:3); w = xi(4:6);
if norm(w) < 1e-6
    T = [eye(3), v * theta; 0 0 0 1];
else
    w_hat = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    R = eye(3) + w_hat * sin(theta) + w_hat^2 * (1 - cos(theta));
    p = (eye(3) - R) * cross(w, v) + w * (w.' * v) * theta;
    T = [R, p; 0 0 0 1];
end
end
