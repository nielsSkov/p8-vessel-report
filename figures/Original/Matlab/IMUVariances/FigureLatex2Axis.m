function FigureLatex2Axis(titletext,xlab,ylab_left,ylab_right,enablelegend,legendtext,labellocation,xlimit,ylimleft,ylimright,font,titlefont,linewidth)
%FigureLatex2Axis(titletext,xlab,ylab_left,ylab_right,enablelegend,legendtext,labellocation,xlimit,ylimleft,ylimright,font,titlefont,linewidth)
% Function that modifies the characteristics of the last figure with the
% parameters set by the user. Just call the function after ploting the
% figure. It is meant to work with two y axes.

hold on
yyaxis left
% Set line width
if linewidth
    lines = findobj(gcf,'Type','Line');
    for i = 1:numel(lines)
        lines(i).LineWidth = linewidth;
    end
end

% Set fonts for axes numbers to Latex
set(gca,'TickLabelInterpreter','latex')

% Change limits and font size of axes numbers
if length(xlimit)==2
    xlim(xlimit)
end

if length(ylimright)==2
    yyaxis right
    ylim(ylimright)
end
if length(ylimleft)==2
    yyaxis left
    ylim(ylimleft)
end

if font
    set(gca,'FontSize',font);
end

% Grid on and minor
grid on
grid minor

% Write label for the axes and set font to Latex and font size
if xlab
    xlabel(xlab,'FontSize',font,'Interpreter','Latex')
end
if ylab_left
    ylabel(ylab_left,'FontSize',font,'Interpreter','Latex')
end
if ylab_right
    yyaxis right
    ylabel(ylab_right,'FontSize',font,'Interpreter','Latex')
end
if titletext
    title(titletext,'FontSize',titlefont,'Interpreter','Latex')
end

% Write legend and set font size and interpreter to Latex
if enablelegend
    if labellocation
        h = legend(legendtext,'Location',labellocation);
    else
        h = legend(legendtext,'Location','SouthEast');
    end
    set(h,'FontSize',font,'Interpreter','Latex');
end

hold off
end