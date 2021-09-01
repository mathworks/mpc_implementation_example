% Copyright 2021 The MathWorks, Inc.
function compare_previous_run(backwardNum)
Simulink.sdi.view;

% Get run IDs for last two runs
runIDs = Simulink.sdi.getAllRunIDs;
runID1 = runIDs(end - backwardNum);
runID2 = runIDs(end);

% Compare runs
Simulink.sdi.compareRuns(runID1,runID2, ...
    'reltol', 1e-3, 'abstol',1e-3, 'timetol',0.5);

end