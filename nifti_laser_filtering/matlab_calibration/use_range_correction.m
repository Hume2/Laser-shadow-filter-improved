function [ errs ] = use_range_correction( lscans_out, fitsurface )
%USE_RANGE_CORRECTION uses fitted correction function to test its
%functionality (only on the target points, 

X_TRESH = 0.00;
LOW_Y_TRESH = -0.34;
HI_Y_TRESH = 0.30;
%%
figure(27);
cla;

errs = [];
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
    
                     
                 
       
    %filter the points and evaluate the corrected values 
    temp_original_points = [lscans_out(i).pointsXY(1,selected_points);...
                            lscans_out(i).pointsXY(2,selected_points)];
                        
    %apply the correction function     
    [ ~ , temp_corrected_points ] = ...
        correct_range_values( lscans_out(i).range_intensity(1,selected_points),...
                              lscans_out(i).range_intensity(2,selected_points),...
                              temp_original_points,...
                              fitsurface);
    
    
    
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
    pause;
end 

end





