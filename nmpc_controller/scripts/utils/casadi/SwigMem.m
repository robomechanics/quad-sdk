function varargout = SwigMem(varargin)
  persistent mem
  mlock
  if (nargin > 1)
    error('Too many input arguments')
  end
  if nargin==0
    if (nargout > 1)
      error('Too many output arguments')
    end
    varargout{1} = mem;
  else
    if (nargout > 0)
      error('Too many output arguments')
    end
    mem = varargin{1};
  end
end
