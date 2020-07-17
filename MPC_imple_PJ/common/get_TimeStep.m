function ts = get_TimeStep(sldd_name)

sldd_obj = Simulink.data.dictionary.open(sldd_name);
data_set_obj = getSection(sldd_obj, 'Design Data');
data_entry = getEntry(data_set_obj, 'TimeStep');
ts_obj = getValue(data_entry);
ts = ts_obj.Value;

end