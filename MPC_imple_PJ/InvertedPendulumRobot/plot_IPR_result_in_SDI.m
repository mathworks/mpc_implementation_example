%% 結果を表示
Simulink.sdi.view;
Simulink.sdi.setSubPlotLayout(3, 1);

RunIDs = Simulink.sdi.getAllRunIDs;
RunID = Simulink.sdi.getRun(RunIDs(end));

sigID = RunID.getSignalIDsByName('ref');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(1, 1, true);
sigHD_children(2).plotOnSubPlot(2, 1, true);

sigID = RunID.getSignalIDsByName('OV');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(1, 1, true);
sigHD_children(2).plotOnSubPlot(2, 1, true);

sigID = RunID.getSignalIDsByName('mv');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(3, 1, true);
sigHD_children(2).plotOnSubPlot(3, 1, true);
