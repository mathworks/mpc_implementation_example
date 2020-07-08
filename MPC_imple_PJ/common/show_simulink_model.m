function show_simulink_model(model_name)

load_system(model_name);

file_path = ['.', filesep, 'cache', filesep, model_name, '.png'];

print('-dpng', ['-s', model_name], file_path);
imshow(file_path);

end