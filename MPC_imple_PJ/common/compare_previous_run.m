Simulink.sdi.view;

% Get run IDs for last two runs
runIDs = Simulink.sdi.getAllRunIDs;
runID1 = runIDs(end - 1);
runID2 = runIDs(end);

% Compare runs
runResult = Simulink.sdi.compareRuns(runID1,runID2, ...
    'reltol',1e-3, 'abstol',1e-3, 'timetol',0.5);
