function csv_load(exp_name)

if nargin==1
    EXP_NAME = exp_name;
else
    EXP_NAME = '20630405';
end
EXP_DATA = {'mech','tracks','ref','vodom','icp','vicon','insorob','leicaref','screws','gpx','lscans','flipper'};
try
rmdir('Data','s')
catch Err
end
current_folder = pwd;



% DATA LOAD
% imports: INS, ODO, GPS, Track (raw ODO), REF
%--------------------------------------------------------------------------------------------------------
data_dir = dir;
mkdir('Data');

for id = 1 : size(data_dir,1) %% cycle through all entries in the current directory (except cerated Data directory)
    
    if data_dir(id).isdir && ( isempty(strfind(data_dir(id).name,'.')) ...
            && isempty(strfind(data_dir(id).name,'..'))) % proceed if the entry is not [. or ..] and is a dir, it is not Data
        cd(data_dir(id).name) % change current folder
        
        temp_dir = dir; % load entries in the current folder
        infilepath = [pwd,'\']; % complete the path
        for j = 1:length(temp_dir) % cycle through all entries
            for k = 1: length(EXP_DATA) % look only for selected files
                if strfind(temp_dir(j).name,char(EXP_DATA(k)))
                    infilelist = temp_dir(j).name;
                else
                    infilelist = 0; % this will cause the cycles below skip
                end
                
                %--------------------------------------------------------------------------------------------------------
               
                
                
                if ~isequal(infilelist,0) && ~isequal(infilepath,0)
                    
                    infilelist = {strrep([infilepath, infilelist], '\', '/')};
                    
                    % Data repository
                    n = 1;
                    nfiles = 1; %numel(infilelist);
                    ptcloud = cell(n,1);
                    % Files
                    for n = 1:nfiles
                        % select inertial data
                        if ~isnan(strfind(infilelist{n}, '_mech_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'ins');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            ins_out =  importdata(infilelist{n},',', headlines);
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_ins_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'ins_out');
                        end
                        % select odometry data
                        if ~isnan(strfind(infilelist{n}, '_odo_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'ins');
                            filename = pathname(start_filename:end);
                            % import data
                            headlines = 0;
                            odo =  importdata(infilelist{n},',', headlines);
                            % reshape odometry data
                            samples = length(odo);
                            odo_out = zeros(samples/14, 14);
                            for i=1:14
                                odo_out(:,i) = downsample(odo(i:end),14);
                            end
                            % save odometry data
                            save([current_folder,sprintf('/Data/nifti_odo_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'odo_out');
                        end
                        % select gps data
                        if ~isnan(strfind(infilelist{n}, '_gpx_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'ins');
                            filename = pathname(start_filename:end);
                            % import data
                            headlines = 0;
                            gps_out =  gpx_load(infilelist{n});
                            % save gps data
                                save([current_folder,sprintf('/Data/nifti_gps_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'gps_out');
                        end
                        
                        % select track data
                        if ~isnan(strfind(infilelist{n}, '_tracks_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'tracks');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            track_out =  importdata(infilelist{n},' ', headlines);
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_track_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'track_out');
                        end
                        
                        % reference data
                        if ~isnan(strfind(infilelist{n}, '_ref_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'ins');
                            filename = pathname(start_filename:end);
                            % import data
                            headlines = 0;
                            ref_out =  importdata(infilelist{n},',', headlines);
                            % swap y axis and 
                            ref_out(:,3) = -ref_out(:,3);
                            ref_out(:,2) = ref_out(:,2) - ref_out(1,2);
                            ref_out(:,3) = ref_out(:,3) - ref_out(1,3);
                            
                            %look for the vpa file, which is necessary to
                            %rotate the reference correctly (in most cases,
                            %the initial angle should be zero, but there is
                            %a possibility, that the robot moved before the
                            %ref. camera was launched...
                            vpa_filename = strrep(infilelist{n},'ref','vpa');
                            if (exist(vpa_filename,'file') ~= 2)
                                
                                warning('ref_process:noVPA','Vpa file %s does not exist, so the initial azimuth of the robot to rotate the reference to will be set to zero.',vpa_filename);
                                init_angle = 0;
                                
                            else                               
                                
                                vpa = importdata(vpa_filename,',', headlines);
                                ref_init_time = ref_out(1,1);
                                vpa_init_row = find( vpa(:,1) >= ref_init_time , 1, 'first');
                                init_angle = vpa(vpa_init_row,16);
                                                        
                            end
    
                            diff_angle = pi*(init_angle - mean(ref_out(1:100,4)))/180;
                            
                            
                            fprintf('Exp. %s diff_angle: %f\n',data_dir(id).name,diff_angle);
                            
                            PN_ref = ([cos(diff_angle) -sin(diff_angle); sin(diff_angle) cos(diff_angle)] * ref_out(:,2:3)');
                            YAW_ref = ref_out(:,4) + repmat(-180*diff_angle/pi,size(ref_out,1),1);
                            
                            overflown_YAW  =  YAW_ref > 180;
                            YAW_ref(overflown_YAW) = YAW_ref(overflown_YAW) - 360;
                            overflown_YAW  =  YAW_ref < -180;
                            YAW_ref(overflown_YAW) = YAW_ref(overflown_YAW) + 360;
                            
                            
                            
                        % save odometry data
                            save([current_folder,sprintf('/Data/nifti_ref_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'ref_out', 'PN_ref', 'YAW_ref');
                        end
                        
                        % select vodom data
                        if ~isnan(strfind(infilelist{n}, '_vodom_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'vodom');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            vodom_out =  importdata(infilelist{n},',', headlines);
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_vodom_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'vodom_out');
                        end
                        
                        % select icp data
                        if ~isnan(strfind(infilelist{n}, '_icp_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'icp');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            icp_out =  importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_icp_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'icp_out');
                        end
                        
                        % select vicon data
                        if ~isnan(strfind(infilelist{n}, '_vicon_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'vicon');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            vicon_out =  importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_vicon_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'vicon_out');
                        end
                        
                        % select vicon data
                        if ~isnan(strfind(infilelist{n}, '_insorob_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'insorob');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            insorob_out =  importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_insorob_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'insorob_out');
                        end
                        
                        % select leica data
                        if ~isnan(strfind(infilelist{n}, '_leicaref_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'leicaref');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            leicaref_out =  importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_leicaref_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'leicaref_out');
                        end
                        
                        % select leica data
                        if ~isnan(strfind(infilelist{n}, '_screws_'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'screws');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            screws_out =  importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_screws_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'screws_out');
                        end
                        
                        % select leica data
                        if ~isnan(strfind(infilelist{n}, 'lscans'))
                            % identify filename
                            pathname = infilelist{n};
                            [lscans_out, track] = csv_load_process_lscans(pathname);
                                                            
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_lscans_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, track)], 'lscans_out');
                        end
                        
                        % select flipper data
                        if ~isnan(strfind(infilelist{n}, 'flipper'))
                            % identify filename
                            pathname = infilelist{n};
                            start_filename = strfind(pathname, 'flipper');
                            filename = pathname(start_filename:end);
                            % import ins data
                            headlines = 0;
                            flippers_out = importdata(infilelist{n},',', headlines);
                                
                            % save ins data
                            save([current_folder,sprintf('/Data/nifti_flippers_%s_%s_track%s.mat', EXP_NAME, data_dir(id).name, filename(end-12:end-4))], 'flippers_out');
                        end
                        
                    end;
                end;
            end
        end
        
        cd ..
    end
end
disp('finished...')
%--------------------------------------------------------------------------------------------------------
%% convert odo
% samples = length(odo);
% odo_out = zeros(samples/14, 14);
% for i=1:14
%     odo_out(:,i) = downsample(odo(i:end),14);
% end
% figure;
% plot(odo_out(:,2)-odo_out(1,2),odo_out(:,3)-odo_out(1,3),'.-r')
%--------------------------------------------------------------------------------------------------------
%% convert gps
% samples = length(gps);
% gps_out = zeros(samples/4, 4);
% gps_out(:,1) = downsample(gps,4);
% gps_out(:,2) = downsample(gps(2:end),4);
% gps_out(:,3) = downsample(gps(3:end),4);
% gps_out(:,4) = downsample(gps(4:end),4);
% figure;
% plot(gps_out(:,2),gps_out(:,3),'*r')

end

function [ data_out, filename ] = csv_load_process_lscans( path )
    cd(path);
    
    all_files = dir;
    scan_files = {};
    
    for i = 1:size(all_files,1)
        if ~isnan(strfind(all_files(i).name,'inso_lscan_'))
            scan_files = [ scan_files ; {all_files(i).name} ];
        end
    end
    
    if isempty(scan_files)
        warn('No laser scan files found in the lscans folder...');
        data_out = [];
        filename = [];
        cd('..');
        return;
    end
    
    num_of_files = size(scan_files,1);
    data_out = struct([]);
    
    for i = 1:num_of_files
        temp_data = importdata(scan_files{i},',',1);
        temp_data = process_lscan_temp(temp_data);
                
        data_out(i,1).range_intensity = temp_data.data;
        data_out(i,1).pointsXY = temp_data.pointsXY;
        data_out(i,1).num_of_points = temp_data.num_of_points;
        data_out(i,1).angle_step = temp_data.angle_step;
        data_out(i,1).angle_max = temp_data.max_angle;
        data_out(i,1).angle_min = temp_data.min_angle;
        data_out(i,1).range_min = temp_data.min_range;
        data_out(i,1).range_max = temp_data.max_range;
        data_out(i,1).timestamp = temp_data.timestamp;
    end
    
    
    
    
    cd('..');
    filename = scan_files{1}((end-33):(end-25));
end

function [data] = process_lscan_temp(data)
%converts the native radial format of the scan to XY format and extracts
%some facts

header = textscan(data.textdata{1},'%d %d %d %f %f %f %f %f','delimiter',',');

time_stamp = double(header{1}) + double(header{2})*0.000000001;
points = double(header{3});
angle_step = double(header{4});
max_angle = double(header{5});
min_angle = double(header{6});
max_range = double(header{7});
min_range = double(header{8});

theta = min_angle;
pointsXY = nan(2,points);


for i = 1:points
    
    if data.data(1,i)>max_range || data.data(1,i)<min_range || data.data(2,i)==0
        theta = theta + angle_step;
        continue;
    end
    
    pointsXY(:,i) = [cos(theta); -sin(theta)] * data.data(1,i);
    theta = theta + angle_step;
end

data.pointsXY = pointsXY;
data.num_of_points = points;
data.angle_step = angle_step;
data.min_angle = min_angle;
data.max_angle = max_angle;
data.max_range = max_range;
data.min_range = min_range;
data.timestamp = time_stamp;

end