%% 結果を表示
Simulink.sdi.view;
Simulink.sdi.setSubPlotLayout(4, 1);

%%
RunIDs = Simulink.sdi.getAllRunIDs;
RunID = Simulink.sdi.getRun(RunIDs(end));

%%
sigID = RunID.getSignalIDsByName('X_Y_theta_V');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(1, 1, true);
sigHD_children(2).plotOnSubPlot(2, 1, true);
sigHD_children(3).plotOnSubPlot(3, 1, true);
sigHD_children(4).plotOnSubPlot(4, 1, true);

sigID = RunID.getSignalIDsByName('refSignal');
sigHD = RunID.getSignal(sigID);
sigHD_children = sigHD.Children;
sigHD_children(1).plotOnSubPlot(1, 1, true);
sigHD_children(2).plotOnSubPlot(2, 1, true);
sigHD_children(3).plotOnSubPlot(3, 1, true);
sigHD_children(4).plotOnSubPlot(4, 1, true);

