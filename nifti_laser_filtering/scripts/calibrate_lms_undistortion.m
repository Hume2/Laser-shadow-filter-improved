% data: 3x541 table of 3 laser scan ranges
% - scan 1: laser plane horizontal, large vertical obstacle directly
%       touching the front of the tracks
% - scan 2: laser plane vertical, no obstacle, on flat ground
% - scan 3: laser plane horizontal, vertical obstacle ca 30 cm in front of
%       the laser

data = csvread('calibration_data.csv');
data(data>20) = NaN;

angle_increment = 0.00872664619237;
angle_min = -2.35619449615;
angle_max = -angle_min;

cloud_orig_x = zeros(size(data));
cloud_orig_z = zeros(size(data));
for i=1:size(data,1)
    for j=1:size(data,2)
        cloud_orig_x(i,j) = data(i,j)*cos(angle_min+(j-1)*angle_increment);
        cloud_orig_z(i,j) = data(i,j)*sin(angle_min+(j-1)*angle_increment);
    end
end

% angle ranges with interesting data (obstacle/flat ground)
scan1_range = 150:400; scan1_range = scan1_range(~isnan(data(1,scan1_range)));
scan2_range = 400:500; scan2_range = scan2_range(~isnan(data(2,scan2_range)));
scan3_range = 190:340; scan3_range = scan3_range(~isnan(data(3,scan3_range)));

% angle ranges containing the correct (non-distorted) part of interesting data
scan1_target=140:160; scan1_target_x = median(data(1,scan1_target).*cos(angle_min+scan1_target*angle_increment)) %0.0852
scan2_target=370:390; scan2_target_z = median(data(2,scan2_target).*sin(angle_min+scan2_target*angle_increment)) %0.1942
scan3_target=185:200; scan3_target_x = median(data(3,scan3_target).*cos(angle_min+scan3_target*angle_increment)) %0.2605

d1 = 0.058; %0.058
c1 = scan1_target_x - d1; %0.032
d2 = 0.169; %0.169
c2 = scan2_target_z - d2; %0.021
d3 = 0.27;
c3 = scan3_target_x - d3;
%{
beta = (c1 * d1 * d1 - c2 * d2 * d2) / (c2 - c1)
alpha = c1 * (beta + d1 * d1)
%}

best_error = [Inf Inf Inf];
error_weights = [1 1 1];
best_alpha = NaN;
best_beta = NaN;

for it=1:10000
    % randomly select 30 points from each interesting part (RANSAC)
    scan1_idxs = unique(sort(scan1_range(randi(length(scan1_range),1,30))));
    scan2_idxs = unique(sort(scan2_range(randi(length(scan2_range),1,30))));
    scan3_idxs = unique(sort(scan3_range(randi(length(scan3_range),1,30))));
    
    if (length(scan1_idxs) < 25 || length(scan2_idxs) < 25 || length(scan3_idxs) < 25)
        continue
    end
    
    A1 = [ones(length(scan1_idxs),1),...
        (data(1,scan1_idxs) - scan1_target_x./cos(angle_min+(scan1_idxs-1)*angle_increment))'
    ];
    A2 = [ones(length(scan2_idxs),1),...
        (data(2,scan2_idxs) - scan2_target_z./sin(angle_min+(scan2_idxs-1)*angle_increment))'
    ];
    A3 = [ones(length(scan3_idxs),1),...
        (data(3,scan3_idxs) - scan3_target_x./cos(angle_min+(scan3_idxs-1)*angle_increment))'
    ];
    A=[A1;A2;A3];
    b = -A(:,2).*[data(1,scan1_idxs).^2, data(2,scan2_idxs).^2, data(3,scan3_idxs).^2]';

    lsq_estimate = A\b;
    alpha = lsq_estimate(1);
    beta = lsq_estimate(2);

    cloud_x = zeros(size(data));
    cloud_z = zeros(size(data));
    data_undistorted = data + alpha ./ (beta + data.^2);

    for i=1:3
        for j=1:541
            cloud_x(i,j) = data_undistorted(i,j)*cos(angle_min+(j-1)*angle_increment);
            cloud_z(i,j) = data_undistorted(i,j)*sin(angle_min+(j-1)*angle_increment);
        end
    end

    error_x = mean(abs(cloud_x(1,scan1_idxs)-scan1_target_x));
    error_z = mean(abs(cloud_z(2,scan2_idxs)-scan2_target_z));
    error_x3 = mean(abs(cloud_x(3,scan3_idxs)-scan3_target_x));
    error = [error_x error_z error_x3];
    
    %if (sum(error.*error_weights) < sum(best_error.*error_weights) && beta > 0)
    if (sum(error.*error_weights) < sum(best_error.*error_weights) && beta > -0.02)
    %if (sum(error.*error_weights) < sum(best_error.*error_weights))
        best_error = error;
        best_alpha = alpha;
        best_beta = beta;
    end
end

alpha = best_alpha
beta = best_beta
best_error

data_undistorted = data + alpha ./ (beta + data.^2);
for i=1:3
    for j=1:541
        cloud_x(i,j) = data_undistorted(i,j)*cos(angle_min+(j-1)*angle_increment);
        cloud_z(i,j) = data_undistorted(i,j)*sin(angle_min+(j-1)*angle_increment);
    end
end

angle_range = angle_min:angle_increment:angle_max;
figure(1); clf;

%%{
subplot(2,3,1);
hold on;
plot(angle_range, cloud_orig_x(1,:)); 
plot(angle_range, cloud_x(1,:), 'r.');
line([-3 3], [scan1_target_x scan1_target_x], 'Color', 'g', 'LineWidth', 2);
line(repmat(min(scan1_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan1_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(min(scan1_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
line(repmat(max(scan1_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
xlim([-1.5 1.5]);
ylim([0.05 0.1]);
hold off;
subplot(2,3,4);
hold on;
plot(angle_range, data(1,:)); 
plot(angle_range, data_undistorted(1,:), 'r.'); 
line(repmat(min(scan1_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan1_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
xlim([-1.5 1.5]);
ylim([0.04 0.2]);
hold off;
%}

%%{
subplot(2,3,2);
hold on;
plot(angle_range, cloud_orig_z(2,:)); 
plot(angle_range, cloud_z(2,:), 'r.');
line([-3 3], [scan2_target_z scan2_target_z], 'Color', 'g', 'LineWidth', 2);
line(repmat(min(scan2_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan2_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(min(scan2_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
line(repmat(max(scan2_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
xlim([0.5 2.3]);
ylim([0.15 0.22]);
hold off;
subplot(2,3,5)
hold on;
plot(angle_range, data(2,:)); 
plot(angle_range, data_undistorted(2,:), 'r.'); 
line(repmat(min(scan2_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan2_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
xlim([0.5 2.3]);
ylim([0.15 0.25]);
hold off;
%}

%%{
subplot(2,3,3);
hold on;
plot(angle_range, cloud_orig_x(3,:)); 
plot(angle_range, cloud_x(3,:), 'r.'); 
line([-3 3], [scan3_target_x scan3_target_x], 'Color', 'g', 'LineWidth', 2);
line(repmat(min(scan3_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan3_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(min(scan3_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
line(repmat(max(scan3_target)*angle_increment+angle_min,1,2), [0 1], 'Color', 'r')
xlim([-1 1]);
ylim([0.2 0.28]);
hold off;
subplot(2,3,6);
hold on;
plot(angle_range, data(3,:)); 
plot(angle_range, data_undistorted(3,:), 'r.'); 
line(repmat(min(scan3_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
line(repmat(max(scan3_range)*angle_increment+angle_min,1,2), [0 1], 'Color', 'g')
xlim([-1 1]);
ylim([0.2 0.35]);
hold off;
%}

figure(2); clf; hold on;
fun_range = 0.05:0.001:1;
plot(fun_range, alpha./(beta+fun_range.^2), 'r');
scatter([d1 d2 d3], [c1 c2 c3], 'r');
plot(fun_range, 0.000497455 ./ (-0.000318182+fun_range.^2));
scatter([0.055 0.25], [0.045 0.008], 'b');
hold off;
