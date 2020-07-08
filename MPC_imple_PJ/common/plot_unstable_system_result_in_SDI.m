%% 結果を表示
Simulink.sdi.view;
Simulink.sdi.setSubPlotLayout(2, 1);

RunIDs = Simulink.sdi.getAllRunIDs;
RunID_1 = Simulink.sdi.getRun(RunIDs(end - 1));
RunID_2 = Simulink.sdi.getRun(RunIDs(end));

sigID = RunID_2.getSignalIDsByName('ref');
sigHD = RunID_2.getSignal(sigID);
sigHD.plotOnSubPlot(1, 1, true);

sigID = RunID_2.getSignalIDsByName('plant_resp');
sigHD = RunID_2.getSignal(sigID);
sigHD.plotOnSubPlot(1, 1, true);
sigHD.set('LineColor', [0, 0, 0]);

sigID = RunID_1.getSignalIDsByName('plant_resp');
sigHD = RunID_1.getSignal(sigID);
sigHD.plotOnSubPlot(1, 1, true);

sigID = RunID_2.getSignalIDsByName('input');
sigHD = RunID_2.getSignal(sigID);
sigHD.plotOnSubPlot(2, 1, true);
