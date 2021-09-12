
% ISOCTAVE  True if the operating environment is octave.
%    Usage: t=isoctave();
% 
%    Returns 1 if the operating environment is octave, otherwise
%    0 (Matlab)
% 
% ---------------------------------------------------------------
function t=isoctave()
%ISOCTAVE  True if the operating environment is octave.
%   Usage: t=isoctave();
%
%   Returns 1 if the operating environment is octave, otherwise
%   0 (Matlab)

if exist('OCTAVE_VERSION')
  % Only Octave has this variable.
  t=1;
else
  t=0;
end;
