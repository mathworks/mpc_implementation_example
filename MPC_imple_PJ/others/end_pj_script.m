%% Init
clear functions;
proj = currentProject;

%% delete temporary files
cd('./cache');
delete('*.*');
try
    rmdir('*','s');
catch
    % Do Nothing
end

cd(proj.RootFolder);
cd('./gen_script');
delete('*.*');

%% Terminate
cd(proj.RootFolder);
clear all;
bdclose all;
clc;
