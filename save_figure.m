function save_figure(h, filename)
%SAVE_FIGURE  Save the given figure as a file
%
%   SAVE_FIGURE(H, FILENAME)
%       (scalar) H       : A handle of the given figure
%       (string) FILENAME: Name of the saved file with extention
%
%   Note: The filename, FILENAME, should have its extention to select the proper
%       file format. (e.g. 'output.jpg') Please refer to the command, SAVEAS, to
%       check the available file formats.
%
%   Examples:
%       save_figure(gcf, 'output.pdf')
%       save_figure(gcf, 'output.png');
%
%   See also saveas, print.

% Make 'Units' same with the paper
fig.unit = get(h, 'Units');
pap.unit = get(h, 'PaperUnits');
set(h, 'Units', pap.unit);

% Make 'PaperSize' same with the figure
fig.rect = get(h, 'Position');
pap.size = get(h, 'PaperSize');
set(h, 'PaperSize', fig.rect(3:4));

% Write a PDF file
saveas(h, filename, format);

% Restore configuration
set(h, 'Units', fig.unit);
set(h, 'PaperSize', pap.size);
