function varargout = SwigStorage(field, varargin)
  persistent dir_mem
  mlock
  narginchk(1,2)
  if isempty(dir_mem)
    dir_mem = struct;
  end
  if nargin==1
    nargoutchk(0,1)
    varargout{1} = dir_mem.(field);
  else
    nargoutchk(0,0)
    if isempty(varargin{1})
      dir_mem = rmfield(dir_mem, field);
    else
      dir_mem.(field) = varargin{1};
    end
  end
end
