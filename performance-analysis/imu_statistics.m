function s = imu_statistics(imu)
% imu_statistics
%
%
%
%
%
%
%

%% ACCELEROMETERS
%%

text_st = {'ACCR X', 'ACCR Y', 'ACCR Z'};

for i=1:3
    
    data = imu.fb(:,i);
    s.a_numpoints(i) = length(data);
    s.a_std(i)       = std(data);
    s.a_mean(i)   = mean(data);     
    s.a_max(i)    = max(data);
    s.a_min(i)    = min(data);
    s.a_median(i) = median(data);
        
    % scale to median for plotting
    medianfreq=data-s.a_median(i);
    
    % 5x Median Absolute Deviation (MAD) criteria
    MAD = median(abs(medianfreq)/0.6745);    
    odl = (abs(medianfreq) > 5*MAD);
    outliers = imu.fb(odl,i);    
    outliers_perc = length(outliers) / length(data);

    %     If outliers are more than 5% of data..
    if  outliers_perc > 0.05
        warning('imu_statistics: there appear to be %d OUTLIERS in %s.\n', length(outliers), cell2mat(text_st(i)) ); 
    end
    
    s.a_outliers(i) = length(outliers);    
end

%% GYROSCOPES
%%

text_st = {'GYRO X', 'GYRO Y', 'GYRO Z'};

for i=1:3
    
    data = imu.wb(:,i);
    s.g_numpoints(i) = length(data);
    s.g_std(i)    = std(data);
    s.g_mean(i)   = mean(data);     
    s.g_max(i)    = max(data);
    s.g_min(i)    = min(data);
    s.g_median(i) = median(data);
        
    % scale to median for plotting
    medianfreq=data-s.g_median(i);
    
    % 5x Median Absolute Deviation (MAD) criteria
    MAD = median(abs(medianfreq)/0.6745);    
    odl = (abs(medianfreq) > 5*MAD);
    outliers = imu.fb(odl,i);    
    outliers_perc = length(outliers) / length(data);

    % If outliers are more than 5% of data..
    if  outliers_perc > 0.05
        warning('imu_statistics: there appear to be %d OUTLIERS in %s.\n', length(outliers), cell2mat(text_st(i)) ); 
    end
    
    s.g_outliers(i) = length(outliers);
end

end