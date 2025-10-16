function ax = create_ax(options)
    arguments
        options.parent = [];
    end
    if isempty(options.parent)
        parent = uifigure(Position=[100,100,1080,400]);
    else
        parent = options.parent;
    end
    ax = uiaxes(parent);
    ax.Box = 'on';
    ax.Position = [10 10 1000 400];
    ax.DataAspectRatio = [ 1 1 1];
    ax.XDir = "reverse";
    ax.YDir = "reverse";
    ax.XAxisLocation = "top";
    ax.YAxisLocation = "right";
    ax.XLabel.String = "X (mm)";
    ax.YLabel.String = "Y (mm)";
    ax.XLim = [-500 4500]; % tank length mm
    ax.YLim = [-300 1200];
    disableDefaultInteractivity(ax);
    hold(ax,"on");
    rectangle(ax,'Position',[0,0,3800,840], ...
                'EdgeColor','r',...
                'LineWidth',2)

end