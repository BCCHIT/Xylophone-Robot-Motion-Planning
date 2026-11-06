function [qBest, costBest] = SelectIKSolution(solutions, qRef, cfg)
% SelectIKSolution
% 从 N x 6 的逆解中选取最优解：
% 1. 先把每组候选解移动到离上一帧最近的 2*pi 等价分支；
% 2. 关节限位内；
% 3. 与上一时刻 qRef 的角度变化最小；
% 4. 轻微惩罚第五轴接近 0 的腕部奇异。

if isempty(solutions)
    error('solutions 为空。');
end

qRef = qRef(:).';
if size(solutions,2) ~= 6
    error('solutions 必须是 N x 6。');
end

% 不能把所有关节角强行包回 [-pi, pi]。实测第 6 轴会从 160 deg
% 连续运动到 200 deg，若包回 ±180 deg 会制造数值跳变。
solutionsContinuous = zeros(size(solutions));
for k = 1:size(solutions,1)
    solutionsContinuous(k,:) = qRef + wrapToPiLocal(solutions(k,:) - qRef);
end

valid = true(size(solutionsContinuous,1),1);
for j = 1:6
    valid = valid & solutionsContinuous(:,j) >= cfg.jointLower(j) & solutionsContinuous(:,j) <= cfg.jointUpper(j);
end

if ~any(valid)
    % 如果限位过严导致全部被删，退回到未过滤解。
    valid = true(size(solutionsContinuous,1),1);
end

idx = find(valid);
costs = inf(size(solutionsContinuous,1),1);

for k = idx.'
    dq = solutionsContinuous(k,:) - qRef;
    jointCost = sum(cfg.jointWeights(:).' .* (dq.^2));
    singularityCost = cfg.singularityPenaltyWeight / (abs(sin(solutionsContinuous(k,5))) + 0.05);
    costs(k) = jointCost + singularityCost;
end

[costBest, bestIdx] = min(costs);
qBest = solutionsContinuous(bestIdx,:);
end

function a = wrapToPiLocal(a)
a = atan2(sin(a), cos(a));
end
