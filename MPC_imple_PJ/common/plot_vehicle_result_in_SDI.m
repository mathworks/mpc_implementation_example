%% 結果を表示
Simulink.sdi.view;
Simulink.sdi.setSubPlotLayout(6, 1);

RunIDs = Simulink.sdi.getAllRunIDs;
RunID = Simulink.sdi.getRun(RunIDs(end));

sigID = RunID.getSignalIDsByName('ref');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(2, 1, true);
sigHD_children(2).plotOnSubPlot(3, 1, true);
sigHD_children(3).plotOnSubPlot(4, 1, true);
sigHD_children(4).plotOnSubPlot(5, 1, true);
sigHD_children(5).plotOnSubPlot(6, 1, true);

sigID = RunID.getSignalIDsByName('y');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(2, 1, true);
sigHD_children(2).plotOnSubPlot(3, 1, true);
sigHD_children(3).plotOnSubPlot(4, 1, true);
sigHD_children(4).plotOnSubPlot(5, 1, true);
sigHD_children(5).plotOnSubPlot(6, 1, true);
