function IMU_T = imu_compare_profiles(imu1, imu2, table_vars)

fields_s = fieldnames(imu1);
fields_m = repmat(fields_s, 1,3);
[n,m] = size(fields_m);
Errors = reshape( (fields_m)', n*m, 1);

IMU_1_c = struct2cell(imu1);
IMU_1_m = cell2mat(IMU_1_c);
IMU1 = reshape(IMU_1_m', n*m, 1);

IMU_2_c = struct2cell(imu2);
IMU_2_m = cell2mat(IMU_2_c(1:n, :));
IMU2 = reshape(IMU_2_m', n*m, 1);

axis = ['X', 'Y', 'Z'];

j = 1;
for i = 1:n*m
    
    str = sprintf('%s %s', cell2mat(Errors(i)), axis( j ) );
    
    j = j + 1;
    if j > 3, j = 1; end
    
    Errors{i} = str;
end

IMU_T = table(Errors, IMU1, IMU2);
IMU_T.Properties.VariableNames{2} = table_vars{1};
IMU_T.Properties.VariableNames{3} = table_vars{2};
