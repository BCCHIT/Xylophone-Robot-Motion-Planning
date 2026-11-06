function [qWaypoints, waypointTimes, cartWaypoints, eventTable] = BuildScheme2JointWaypoints(notes, beats, BPM, keyMap, cfg)
% BuildScheme2JointWaypoints
% 方案2使用实测关节角作为敲击点，并在关节空间做 LFPB 规划。

quarter = 60 / BPM;
durations = beats(:) * quarter;
notes = notes(:);

if numel(notes) ~= numel(durations)
    error('notes and beats must have the same length.');
end

qWaypoints = [];
cartWaypoints = [];
waypointTimes = [];
eventType = {};
eventNote = [];
eventIndex = [];
currentTime = 0;
qPrev = cfg.scheme2.qSeed(:).';

for i = 1:numel(notes)
    note = notes(i);
    if note < 1 || note > size(keyMap.qHit, 1)
        error('Note %d at index %d is outside the 1~15 key map.', note, i);
    end

    qHitMeasured = keyMap.qHit(note, :);
    qHitMeasured = qPrev + wrapToPiLocal(qHitMeasured - qPrev);

    [qSafe, qHit, qLift, pSafe, pHit, pLift] = buildOneNotePostures(qHitMeasured, cfg);

    T = durations(i);
    tSafe = currentTime;
    tHit = currentTime + cfg.scheme2.downRatio * T;
    tLift = currentTime + (cfg.scheme2.downRatio + cfg.scheme2.upRatio) * T;

    qWaypoints = [qWaypoints; qSafe; qHit; qLift]; %#ok<AGROW>
    cartWaypoints = [cartWaypoints; pSafe; pHit; pLift]; %#ok<AGROW>
    waypointTimes = [waypointTimes; tSafe; tHit; tLift]; %#ok<AGROW>

    eventType = [eventType; {'safe'}; {'hit'}; {'lift'}]; %#ok<AGROW>
    eventNote = [eventNote; note; note; note]; %#ok<AGROW>
    eventIndex = [eventIndex; i; i; i]; %#ok<AGROW>

    currentTime = currentTime + T;
    qPrev = qLift;
end

qWaypoints = [qWaypoints; qWaypoints(end,:)];
cartWaypoints = [cartWaypoints; cartWaypoints(end,:)];
waypointTimes = [waypointTimes; currentTime];
eventType = [eventType; {'end_hold'}];
eventNote = [eventNote; notes(end)];
eventIndex = [eventIndex; numel(notes)];

if cfg.scheme2.finalReturnTime > 0
    qWaypoints = [qWaypoints; qWaypoints(1,:)];
    T0 = Fkine(cfg.Xi, qWaypoints(end,:), cfg.g0);
    cartWaypoints = [cartWaypoints; T0(1:3,4).'];
    waypointTimes = [waypointTimes; currentTime + cfg.scheme2.finalReturnTime];
    eventType = [eventType; {'return_start'}];
    eventNote = [eventNote; NaN];
    eventIndex = [eventIndex; NaN];
end

[waypointTimes, ia] = unique(waypointTimes, 'stable');
qWaypoints = qWaypoints(ia,:);
cartWaypoints = cartWaypoints(ia,:);
eventType = eventType(ia);
eventNote = eventNote(ia);
eventIndex = eventIndex(ia);

eventTable = table(waypointTimes, eventIndex, eventNote, eventType, ...
    'VariableNames', {'time','noteIndex','note','event'});
end

function [qSafe, qHit, qLift, pSafe, pHit, pLift] = buildOneNotePostures(qHitMeasured, cfg)
TMeasured = Fkine(cfg.Xi, qHitMeasured, cfg.g0);
R = TMeasured(1:3,1:3);
pMeasured = TMeasured(1:3,4);

pSafe = pMeasured + [0; 0; cfg.scheme2.approachHeight];
pHit = pMeasured + [0; 0; -cfg.scheme2.strikeDepth];
pLift = pMeasured + [0; 0; cfg.scheme2.reboundHeight];

qSafe = solveNear([R, pSafe; 0 0 0 1], qHitMeasured, cfg);
qHit = solveNear([R, pHit; 0 0 0 1], qSafe, cfg);
qLift = solveNear([R, pLift; 0 0 0 1], qHit, cfg);

pSafe = pSafe.';
pHit = pHit.';
pLift = pLift.';
end

function q = solveNear(T, qRef, cfg)
sols = Ikine6s_all(T, cfg.config).';
if isempty(sols)
    error('Scheme 2 safe/hit/lift posture is unreachable.');
end
q = SelectIKSolution(sols, qRef, cfg);
end

function a = wrapToPiLocal(a)
a = atan2(sin(a), cos(a));
end
