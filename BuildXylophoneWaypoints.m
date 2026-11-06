function [waypointsBase, waypointTimes, eventTable] = BuildXylophoneWaypoints(notes, beats, BPM, keyMap, cfg)
% BuildXylophoneWaypoints
% 将音符/节拍转换为机器人末端法兰点的关键点序列。
%
% keyMap.keyCentersBase 存的是锤头接触点，不是法兰点。
% 若 cfg.hammerOffsetTool 非零，则本函数会根据固定末端姿态把锤头点转换为法兰点：
% p_flange = p_head - R_target * hammerOffsetTool。

quarter = 60 / BPM;
durations = beats(:) * quarter;
notes = notes(:);

if numel(notes) ~= numel(durations)
    error('notes 与 beats 长度不一致。');
end

R = cfg.R_verticalHit;
toolOffsetBase = (R * cfg.hammerOffsetTool).';

waypointsBase = [];
waypointTimes = [];
eventType = {};
eventNote = [];
eventIndex = [];

currentTime = 0;

for i = 1:numel(notes)
    note = notes(i);
    if note < 1 || note > size(keyMap.keyCentersBase,1)
        error('第 %d 个音符 note=%d 超出 keyMap 范围。', i, note);
    end

    keyHead = keyMap.keyCentersBase(note,:);   % 锤头在琴键表面的接触点

    safeHead = keyHead + [0, 0, cfg.approachHeight];
    hitHead  = keyHead + [0, 0, -cfg.strikeDepth];
    liftHead = keyHead + [0, 0, cfg.reboundHeight];

    safeFlange = safeHead - toolOffsetBase;
    hitFlange  = hitHead  - toolOffsetBase;
    liftFlange = liftHead - toolOffsetBase;

    T = durations(i);
    tSafe = currentTime;
    tHit  = currentTime + cfg.downRatio*T;
    tLift = currentTime + (cfg.downRatio + cfg.upRatio)*T;

    waypointsBase = [waypointsBase; safeFlange; hitFlange; liftFlange];
    waypointTimes = [waypointTimes; tSafe; tHit; tLift];

    eventType = [eventType; {'safe'}; {'hit'}; {'lift'}];
    eventNote = [eventNote; note; note; note];
    eventIndex = [eventIndex; i; i; i];

    currentTime = currentTime + T;
end

% 补一个“保持到最后一个音符结束”的点，避免最后一个音提前结束。
waypointsBase = [waypointsBase; waypointsBase(end,:)];
waypointTimes = [waypointTimes; currentTime];
eventType = [eventType; {'end_hold'}];
eventNote = [eventNote; notes(end)];
eventIndex = [eventIndex; numel(notes)];

% 可选：回到第一音准备点上方，便于安全收尾。
if cfg.finalReturnTime > 0
    homePoint = waypointsBase(1,:);
    homePoint(3) = max(homePoint(3), max(waypointsBase(:,3)) + 60);
    waypointsBase = [waypointsBase; homePoint];
    waypointTimes = [waypointTimes; currentTime + cfg.finalReturnTime];
    eventType = [eventType; {'return_home'}];
    eventNote = [eventNote; NaN];
    eventIndex = [eventIndex; NaN];
end

% 删除可能出现的重复时间点，只保留后一个。
[waypointTimes, ia] = unique(waypointTimes, 'stable');
waypointsBase = waypointsBase(ia,:);
eventType = eventType(ia);
eventNote = eventNote(ia);
eventIndex = eventIndex(ia);

eventTable = table(waypointTimes, eventIndex, eventNote, eventType, ...
    'VariableNames', {'time','noteIndex','note','event'});
end
