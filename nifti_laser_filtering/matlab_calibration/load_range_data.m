clear all;
%% select laser scan files
[FileName,PathName] = uigetfile('*.mat','Select the MATLAB code file','MultiSelect','on');

length_original = []; 
intensities = [];
length_deltas = [];

for i = 1:size(FileName,2)
    load([PathName FileName{i}]);
    
    [ temp_length_original, temp_intensities, temp_length_deltas  ] = eval_range_intensity_correction( lscans_out );

    length_original = [length_original, temp_length_original]; 
    intensities = [intensities, temp_intensities];
    length_deltas = [length_deltas, temp_length_deltas];
end

figure;
plot3(length_original(1:5:end), intensities(1:5:end), length_deltas(1:5:end),'*');
grid on;
xlabel('Range[m]');
ylabel('Intensity[-]');
zlabel('Correction[m]');



figure;
fitsurface=fit([length_original',intensities'],length_deltas', 'poly55','Normalize','on');
plot(fitsurface, [length_original',intensities'],length_deltas');
xlabel('Range[m]');
ylabel('Intensity[-]');
zlabel('Correction[m]');
fitsurface %#ok


%% verify

for i = 1:size(FileName,2)
    load([PathName FileName{i}]);
    use_range_correction( lscans_out, fitsurface );
end



