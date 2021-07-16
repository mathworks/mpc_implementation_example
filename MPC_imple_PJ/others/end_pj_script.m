%% Init
clear functions;
proj = currentProject;

%% delete temporary files
cd(proj.RootFolder + filesep + "cache");
delete('*.*');
try
    rmdir('*','s');
catch
    % Do Nothing
end

create_text_file(pwd, 'readme_cache.txt', ...
    'This folder is for temporary files.');

cd(proj.RootFolder + filesep + "gen_script");
delete('*.*');

create_text_file(pwd, 'readme_gen_script.txt', ...
    'This folder is for temporary files.');

%% Terminate
cd(proj.RootFolder);

allDocs = matlab.desktop.editor.getAll;
allDocs.close;
clear all;
bdclose all;
clc;
