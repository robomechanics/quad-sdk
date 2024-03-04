classdef  MatlabSwigIterator < SwigRef
    %MATLABSWIGITERATOR 
    %
    %   = MATLABSWIGITERATOR()
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(5, self);
          self.SwigClear();
        end
    end
    function varargout = value(self,varargin)
    %VALUE 
    %
    %  mxArray * = VALUE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(6, self, varargin{:});
    end
    function varargout = incr(self,varargin)
    %INCR 
    %
    %  MatlabSwigIterator = INCR(self, size_t n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(7, self, varargin{:});
    end
    function varargout = decr(self,varargin)
    %DECR 
    %
    %  MatlabSwigIterator = DECR(self, size_t n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(8, self, varargin{:});
    end
    function varargout = distance(self,varargin)
    %DISTANCE 
    %
    %  ptrdiff_t = DISTANCE(self, MatlabSwigIterator x)
    %
    %
      [varargout{1:nargout}] = casadiMEX(9, self, varargin{:});
    end
    function varargout = equal(self,varargin)
    %EQUAL 
    %
    %  bool = EQUAL(self, MatlabSwigIterator x)
    %
    %
      [varargout{1:nargout}] = casadiMEX(10, self, varargin{:});
    end
    function varargout = copy(self,varargin)
    %COPY 
    %
    %  MatlabSwigIterator = COPY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(11, self, varargin{:});
    end
    function varargout = next(self,varargin)
    %NEXT 
    %
    %  mxArray * = NEXT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(12, self, varargin{:});
    end
    function varargout = previous(self,varargin)
    %PREVIOUS 
    %
    %  mxArray * = PREVIOUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(13, self, varargin{:});
    end
    function varargout = advance(self,varargin)
    %ADVANCE 
    %
    %  MatlabSwigIterator = ADVANCE(self, ptrdiff_t n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(14, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, MatlabSwigIterator x)
    %
    %
      [varargout{1:nargout}] = casadiMEX(15, self, varargin{:});
    end
    function varargout = ne(self,varargin)
    %NE 
    %
    %  bool = NE(self, MatlabSwigIterator x)
    %
    %
      [varargout{1:nargout}] = casadiMEX(16, self, varargin{:});
    end
    function varargout = TODOincr(self,varargin)
    %TODOINCR 
    %
    %  MatlabSwigIterator = TODOINCR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(17, self, varargin{:});
    end
    function varargout = TODOdecr(self,varargin)
    %TODODECR 
    %
    %  MatlabSwigIterator = TODODECR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(18, self, varargin{:});
    end
    function varargout = plus(self,varargin)
    %PLUS 
    %
    %  MatlabSwigIterator = PLUS(self, ptrdiff_t n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(19, self, varargin{:});
    end
    function varargout = minus(self,varargin)
    %MINUS 
    %
    %  ptrdiff_t = MINUS(self, MatlabSwigIterator x)
    %  MatlabSwigIterator = MINUS(self, ptrdiff_t n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(20, self, varargin{:});
    end
    function self = MatlabSwigIterator(varargin)
    %MATLABSWIGITERATOR 
    %
    %   = MATLABSWIGITERATOR()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end
