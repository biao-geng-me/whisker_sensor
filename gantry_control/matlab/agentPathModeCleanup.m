function agentPathModeCleanup(app)
% agentPathModeCleanup Best-effort cleanup for PathAgent modes.

try
    if ~isempty(app.CC1) && ~isempty(app.CC1.Car)
        app.CC1.Car.stopPathTracking();
    end
catch
end

try
    if ~isempty(app.CC2) && ~isempty(app.CC2.Car)
        app.CC2.Car.stopPathTracking();
    end
catch
end

try
    app.CC2.Car.poll_gamepad = 0;
    app.CC2.Car.poll_keyboard = 0;
catch
end

try
    app.CC1.hArrow.Visible = 'off';
catch
end

try
    app.CC2.hArrow.Visible = 'off';
catch
end

try
    app.WA.is_recording = false;
catch
end
