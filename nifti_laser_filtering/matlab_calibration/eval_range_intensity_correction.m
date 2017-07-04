function [ length_original, intensities, length_deltas  ] = eval_range_intensity_correction( lscans_out )
%EVAL_RENGE_INTENSITY_CORRECTION 

X_TRESH = 0.00;
LOW_Y_TRESH = -0.34;
HI_Y_TRESH = 0.30;
%%
figure(27);
cla;

length_deltas = [];
length_original = [];
intensities = [];

for i = 1:size(lscans_out,1)
    
    %filter irrelevant points out    
    points_in_front_of_laser = lscans_out(i).pointsXY(1,:) >= X_TRESH;
    points_in_the_calib_target = (lscans_out(i).pointsXY(2,:) >= LOW_Y_TRESH )&( ...
        lscans_out(i).pointsXY(2,:) <= HI_Y_TRESH);
    selected_points = points_in_front_of_laser & points_in_the_calib_target & ...
        ~isnan(lscans_out(i).pointsXY(2,:));
    
    %find both ends of the target to draw a line
    first_points = find(selected_points,5,'first');
    last_points = find(selected_points,5,'last');
    
    %line, so polynomial n = 1
    line_coefs = polyfit(lscans_out(i).pointsXY(2,[first_points last_points]),...
                         lscans_out(i).pointsXY(1,[first_points last_points]),1);
    
                     
    
                     
    %find correcting length that moves each point the ideal line
    % the line is x=ky+q
    % the new point location is [x,y] = [lx',ly'] where x',y' are original
    % points and l is the length correction
    % l = q/(x'-ky')
    
    k = line_coefs(1);
    q = line_coefs(2);
    
    l = q ./ (lscans_out(i).pointsXY(1,selected_points) ...
             - k*lscans_out(i).pointsXY(2,selected_points));
    
    
    
    %filter the points and evaluate the corrected values 
    temp_original_points = [lscans_out(i).pointsXY(1,selected_points);...
                            lscans_out(i).pointsXY(2,selected_points)];
                        
    temp_corrected_points = temp_original_points.*repmat(l,2,1);     
    
    
    temp_original_lengths = lscans_out(i).range_intensity(1,selected_points);
    temp_corrected_lengths = sqrt(sum(temp_corrected_points.^2,1));
    temp_length_delta = temp_corrected_lengths - temp_original_lengths;
    temp_intensities = lscans_out(i).range_intensity(2,selected_points);
    
    
    length_deltas = [length_deltas, temp_length_delta];
    length_original = [length_original, temp_original_lengths];
    intensities = [intensities, temp_intensities];
    
    %DEBUG: Plot the stuff
    cla;
    plot(-temp_original_points(2,:),...
          temp_original_points(1,:),'*');
    axis equal;
    hold on;
    grid on;
    plot(-temp_corrected_points(2,:),...
          temp_corrected_points(1,:),'*');
    
    
    plot(0,0,'rd')
    plot(-lscans_out(i).pointsXY(2,first_points),...
          lscans_out(i).pointsXY(1,first_points),'rd');
    plot(-lscans_out(i).pointsXY(2,last_points),...
          lscans_out(i).pointsXY(1,last_points),'rd');
      
    plot(-[lscans_out(i).pointsXY(2,first_points(1)),...
           lscans_out(i).pointsXY(2,last_points(end))],...
          [polyval(line_coefs,lscans_out(i).pointsXY(2,first_points(1))),...
           polyval(line_coefs,lscans_out(i).pointsXY(2,last_points(end)))],...
            'r-');  
      
    drawnow;  
    
end 

end

