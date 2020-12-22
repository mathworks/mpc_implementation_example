function set_slddVal(sldd_name, val_name, val)

sldd_obj = Simulink.data.dictionary.open(sldd_name);
data_set_obj = getSection(sldd_obj, 'Design Data');
data_entry = getEntry(data_set_obj, val_name);

valObj = getValue(data_entry);
valObj.Value = val;
setValue(data_entry, valObj);

saveChanges(sldd_obj);

end