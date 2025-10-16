s = serialportfind();
for i=1:numel(s)
    delete(s(i)); % this is necessary to really disconnect the serail port.
end
clear s