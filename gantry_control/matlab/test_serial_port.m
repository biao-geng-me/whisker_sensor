% This script is for troubleshooting serial port communication with the microcontroller.
% It reads lines of data, logs their arrival times, and plots the inter-arrival intervals to check for timing consistency.
% set portname and baudrate to match your setup, then run this script while the microcontroller is sending data.

clc; clear
clear all; % clear persistent variables in printLine

portname = "COM18";
baudrate = 2000000;

% clear if already connected
s=serialportfind('Port',portname);
if ~isempty(s)
    delete(s);
end

s = serialport(portname, baudrate);
flush(s);

% Shared state via persistent or appdata - use guidata-style via base workspace
setappdata(0, 'serialLog', []);
setappdata(0, 'serialDone', false);

% config call back
% configureCallback(s, "terminator", @(src,evt) printLine(src,evt)); % for clearcore controller use terminator
nbytes_per_sample = 18 * 4 + 4 + 1; % 18 float32 + 1 float32 marker + 1 char ('\n') % use byte count for sensor arduino
align_data_read(s, nbytes_per_sample);
configureCallback(s, "byte",nbytes_per_sample, @(src,evt) printLine(src,evt)); 

% Wait until collection is done
disp("Collecting data...");
while ~getappdata(0, 'serialDone')
    pause(0.1);
end

% Retrieve log and plot
log = getappdata(0, 'serialLog');
if length(log) > 1
    intervals = diff(log) * 1000; % convert to ms
    figure;
    subplot(2,1,1);
    plot(intervals);
    yline(5, 'r--', '5ms (200Hz)');
    xlabel('Packet #'); ylabel('Interval (ms)');
    title(sprintf('%s inter-packet arrival intervals',portname));
    grid on;

    subplot(2,1,2);
    histogram(intervals, 50);
    xlabel('Interval (ms)'); ylabel('Count');
    stats = sprintf("Mean: %.2f ms | Std: %.2f ms | Min: %.2f ms | Max: %.2f ms\n", ...
        mean(intervals), std(intervals), min(intervals), max(intervals));
    fprintf('%s\n', stats);
    title(sprintf('%s interval distribution\n%s', portname, stats));
    grid on;
end


% serial call back
function printLine(src, ~)
    persistent count startTime
    N = 500;         % number of lines to collect
    T = 10;          % fallback timeout in seconds

    if isempty(count)
        count = 0;
        startTime = datetime('now');
    end

    line = readline(src);
    count = count + 1;

    % Log arrival time
    log = getappdata(0, 'serialLog');
    log(end+1) = posixtime(datetime('now'));
    setappdata(0, 'serialLog', log);

    % Print with timestamp
    fprintf('%s [%d] %s\n', datetime('now','Format','HH:mm:ss.SSS'), count, line);

    % Stop after N lines or T seconds
    elapsed = seconds(datetime('now') - startTime);
    if count >= N || elapsed >= T
        configureCallback(src, "off");
        setappdata(0, 'serialDone', true);
        fprintf("Done. Collected %d lines in %.2f seconds.\n", count, elapsed);
    end
end

% helper function for arduino sensor array
function align_data_read(s, nbytes)
    % align data read
    % the value 2024 is a marker to indicate the start of a data frame (see HX711_array Arduino code)
    % this while loop is necessary because the line feed \n can randomly appear in the binary data stream

    tic
    flush(s);

    tmpv =0;
    fprintf('Searching frame start\n');
    ncount = 0;
    while(tmpv~=2024)
        fprintf('.')
        ncount = ncount + 1;
        readline(s);
        tmpv = read(s,1,'single');
        if mod(ncount,100)==0
            fprintf('\n');
        end
    end

    % clear old frames from serial buffer
    fprintf('Clearing buffer\n');
    while(s.NumBytesAvailable>=nbytes)
        read(s,nbytes,'char');
    end
    fprintf('%g seconds to find sample start\n',toc);
end