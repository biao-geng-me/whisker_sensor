function params = get_params(filename,re)

    arguments
        filename (1,1) string
        re % cell array of regex patterns to match
    end

    nre = length(re);
    params = cell(nre,1);
    for i=1:nre
        % Extract value
        rex = re{i};
        rpm_match = regexp(filename, rex, 'tokens');
        if ~isempty(rpm_match)
            params{i} = rpm_match{1}{1};
        else
            params{i} = NaN; % Handle case if not found
        end
    end
    