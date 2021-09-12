function [] = setupTestingPaths( )

    addpath(genpath(pwd));
    addpath(genpath([pwd 'auxFiles']));
    addpath(genpath([pwd 'data']));
    addpath(genpath([pwd 'tests']));
    
    if isoctave 
		addpath('../../interfaces/octave/');
	else
		addpath('../../interfaces/matlab/');
	end

end
