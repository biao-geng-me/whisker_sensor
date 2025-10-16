function time_string = ms_to_hms_string(ms)
%MS_TO_HMS_STRING Converts milliseconds to a formatted H:MM:SS.mmm string.
%
%   TIME_STRING = MS_TO_HMS_STRING(MS)
%   Converts an input duration in milliseconds (MS) into a string formatted as
%   HH:MM:SS.mmm, where:
%   - HH are hours (padded to two digits)
%   - MM are minutes (padded to two digits)
%   - SS.mmm are seconds and milliseconds (padded to two digits for seconds,
%     and fixed to three decimal places for milliseconds)
%
%   Example:
%   >> ms_to_hms_string(3661234)
%   ans =
%       '01:01:01.234'
%

    % 1. Convert milliseconds to total seconds (float)
    totalSeconds = ms / 1000.0;

    % 2. Calculate Hours (H)
    H = floor(totalSeconds / 3600);
    
    % Get remaining seconds after hours are removed
    remSeconds = mod(totalSeconds, 3600);

    % 3. Calculate Minutes (M)
    M = floor(remSeconds / 60);

    % 4. Calculate Seconds (S) including milliseconds
    S = mod(remSeconds, 60);
    
    % 5. Format the string
    % %02d: Zero-padded integer to 2 digits for H and M
    % %06.3f: Fixed-point float, 6 total characters wide (including the decimal point), 
    %         with 3 digits after the decimal point for milliseconds. This ensures
    %         seconds like '01.234' are correctly padded.
    time_string = sprintf('%02d:%02d:%06.3f', H, M, S);
end
