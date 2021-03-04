function colorOutput = cmuColor(printType)
% Returns a 3x1 vector of RBG values for the requested official CMU color 
% given in 'printType'. No argument will return 'red-web'. The currently 
% supported colors areprimary colors 'red-web', 'red-print', 'gray', and 
% 'dark-gray', and secondary colors 'gold', 'teal', 'blue', 'green', and 
% 'dark-green'. 'random' will select one of these colors at random.

if nargin == 0
    printType = 'web';
end

colorList = {'red-web', 'red-print', 'gray', 'dark-gray', 'gold',...
    'teal', 'blue', 'green', 'dark-green'};

if strcmp(printType, 'random')
    printType = colorList{randi(length(colorList))};
end

switch printType
    case 'red-web'
        colorOutput = 1/255*[187 0 0];
    case 'red-print'
        colorOutput = 1/255*[176 28 46];
    case 'gray'
        colorOutput = 1/255*[244 244 244];
    case 'dark-gray'
        colorOutput = 1/255*[102 102 102];
    case 'gold'
        colorOutput = 1/255*[170 102 0];
    case 'teal'
        colorOutput = 1/255*[0 102 119];
    case 'blue'
        colorOutput = 1/255*[34 68 119];
    case 'green'
        colorOutput = 1/255*[0 136 85];
    case 'dark-green'
        colorOutput = 1/255*[34 68 51];
    case 'colororder'
        colorOutput = [1/255*[187 0 0];
                       1/255*[34 68 119];
                       1/255*[0 136 85];
                       1/255*[170 102 0];
                       1/255*[0 102 119];
                       1/255*[34 68 51];];
end