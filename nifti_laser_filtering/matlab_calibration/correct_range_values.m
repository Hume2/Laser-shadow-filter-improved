function [ corrected_ranges, corrected_positions ] = correct_range_values( range_orig, intensity, position_orig, fitsurface)
%CORRECT_RANGE_VALUES evaluates correct range and also the position

corrected_ranges = nan(size(range_orig));
corrected_positions = nan(size(position_orig));

for i = 1:size(range_orig,2)
    
    temp_delta_range = fitsurface([range_orig(i),intensity(i)]); 
    temp_corrected_range = range_orig(i)+temp_delta_range;
    corrected_ranges(i) = temp_corrected_range;
    
    pos_vector_length_correction = temp_corrected_range / range_orig(i);
    corrected_positions(:,i) = position_orig(:,i)*pos_vector_length_correction;
end




end

