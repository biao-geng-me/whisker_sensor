function handleAgentConnectionFailure(app, context, ME)
% handleAgentConnectionFailure Best-effort response to a lost Python server connection.

warning('AgentConnectionError:Disconnected', ...
    '%s failed: %s', context, ME.message);

try
    if ~isempty(app.server_config_window) && isvalid(app.server_config_window) ...
            && isvalid(app.server_config_window.UIFigure)
        app.server_config_window.setServerRunning(false, ...
            sprintf('Server connection lost during %s.', context));
    end
catch
end

try
    if ~isempty(app.net)
        app.net.shutdown();
    end
catch
end

try
    app.net = [];
catch
end
