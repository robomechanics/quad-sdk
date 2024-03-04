classdef  Slice < casadi.PrintableCommon
    %SLICE [INTERNAL] 
    %
    %
    %Class representing a  Slice.
    %
    %Note that Python or Octave do not need to use this class. They can 
    %just use
    % slicing utility from the host language ( M[0:6] in Python, 
    %M(1:7) )
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_13
    %
    %C++ includes: slice.hpp
    %
    %
  methods
    function v = start(self)
      v = casadiMEX(190, self);
    end
    function v = stop(self)
      v = casadiMEX(191, self);
    end
    function v = step(self)
      v = casadiMEX(192, self);
    end
    function varargout = all(self,varargin)
    %ALL [INTERNAL] 
    %
    %  [int] = ALL(self)
    %  [int] = ALL(self, int len, bool ind1)
    %  [int] = ALL(self, Slice outer, int len)
    %
    %Get a vector of indices (nested slice)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L75
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L129-L137
    %
    %
    %
    %.......
    %
    %::
    %
    %  ALL(self)
    %
    %
    %
    %[INTERNAL] 
    %Get a vector of indices.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L69
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L90-L98
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  ALL(self, int len, bool ind1)
    %
    %
    %
    %[INTERNAL] 
    %Get a vector of indices.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L72
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L100-L102
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  ALL(self, Slice outer, int len)
    %
    %
    %
    %[INTERNAL] 
    %Get a vector of indices (nested slice)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L75
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L129-L137
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(193, self, varargin{:});
    end
    function varargout = size(self,varargin)
    %SIZE [INTERNAL] 
    %
    %  size_t = SIZE(self)
    %
    %Get number of elements.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L78
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L104-L109
    %
    %
    %
      out = casadiMEX(194, self, varargin{:});
      if nargout<=1
        varargout{1}=out;
      else
        nargoutchk(length(out),length(out))
        for i=1:nargout
          varargout{i} = out(i);
        end
      end
    end
    function varargout = is_empty(self,varargin)
    %IS_EMPTY [INTERNAL] 
    %
    %  bool = IS_EMPTY(self)
    %
    %Check if slice is empty.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L81
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L111-L113
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(195, self, varargin{:});
    end
    function varargout = is_scalar(self,varargin)
    %IS_SCALAR [INTERNAL] 
    %
    %  bool = IS_SCALAR(self, int len)
    %
    %Is the slice a scalar.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L84
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L139-L144
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(196, self, varargin{:});
    end
    function varargout = scalar(self,varargin)
    %SCALAR [INTERNAL] 
    %
    %  int = SCALAR(self, int len)
    %
    %Get scalar (if is_scalar)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L87
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L146-L150
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(197, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, Slice other)
    %
    %
      [varargout{1:nargout}] = casadiMEX(198, self, varargin{:});
    end
    function varargout = ne(self,varargin)
    %NE 
    %
    %  bool = NE(self, Slice other)
    %
    %
      [varargout{1:nargout}] = casadiMEX(199, self, varargin{:});
    end
    function varargout = apply(self,varargin)
    %APPLY [INTERNAL] 
    %
    %  Slice = APPLY(self, int len, bool ind1)
    %
    %Apply concrete length.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L98
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L66-L88
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(200, self, varargin{:});
    end
    function varargout = minus(self,varargin)
    %MINUS 
    %
    %  Slice = MINUS(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(201, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
    %MTIMES 
    %
    %  Slice = MTIMES(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(202, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME [INTERNAL] 
    %
    %  char = TYPE_NAME(self)
    %
    %Get name of the class.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L107
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L107-L107
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(203, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP [INTERNAL] 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %Print a description of the object.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L110
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L115-L127
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(204, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR [INTERNAL] 
    %
    %  char = STR(self, bool more)
    %
    %Get string representation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L113
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L113-L117
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(205, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO [INTERNAL] 
    %
    %  struct = INFO(self)
    %
    %Obtain information
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L120
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L120-L122
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(206, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE [INTERNAL] 
    %
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %Serialize an object.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_14
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L127
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L292-L296
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(207, self, varargin{:});
    end
    function self = Slice(varargin)
    %SLICE 
    %
    %  new_obj = SLICE()
    %  new_obj = SLICE(int i, bool ind1)
    %  new_obj = SLICE(int start, int stop, int step)
    %  new_obj = SLICE(int start, int stop, int step)
    %  new_obj = SLICE(int start, int stop, int step)
    %  new_obj = SLICE(int start, int stop, int step)
    %
    %
    %.......
    %
    %::
    %
    %  SLICE()
    %
    %
    %
    %[INTERNAL] 
    %Default constructor - all elements.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L57
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L32-L33
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SLICE(int i, bool ind1)
    %
    %
    %
    %[INTERNAL] 
    %A single element (explicit to avoid ambiguity with IM overload.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L60
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L35-L42
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SLICE(int start, int stop, int step)
    %
    %
    %
    %[INTERNAL] 
    %A slice.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.hpp#L63
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/slice.cpp#L44-L45
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  SLICE(int start, int stop, int step)
    %  SLICE(int start, int stop, int step)
    %  SLICE(int start, int stop, int step)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(209, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(210, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Slice = DESERIALIZE(casadi::DeserializingStream & s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(208, varargin{:});
    end
  end
end
