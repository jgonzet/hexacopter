function [] = save_run(folder, name, Angles, AttCommands, AngleRates, ff, ZLMN,Ts)


%% Variables of interest
run.time = Angles.time;
run.angles = Angles.signals.values;
run.refs = AttCommands.signals.values;
run.AngleRates = AngleRates.signals.values;
run.FF = ff.signals.values;
run.ZLMN = ZLMN.signals.values;


%% Save
cHeader = {'time_sec' 'roll_deg' 'pitch_deg' 'yaw_deg' 'rollref_deg' 'pitchref_deg' 'yawref_deg' 'rollspeed_degperrsec' 'pitchspeed_degperrsec' 'yawspeed_degperrsec' 'f1_kg' 'f2_kg' 'f3_kg' 'f4_kg' 'f5_kg' 'f6_kg' 'Z' 'L' 'M' 'N'}; %header

commaHeader = [cHeader;repmat({','},1,numel(cHeader))]; %insert commas
commaHeader = commaHeader(:)';
textHeader = cell2mat(commaHeader); %cHeader in text with commas

filename=[folder '/' name '.csv'];
%write header to file
fid = fopen(filename,'w'); 
fprintf(fid,'%s\n',textHeader);
fclose(fid);
%write data to end of file
dlmwrite(filename, [run.time run.angles run.refs run.AngleRates run.FF run.ZLMN], '-append');

end