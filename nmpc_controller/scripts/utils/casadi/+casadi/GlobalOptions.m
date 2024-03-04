classdef  GlobalOptions < SwigRef
    %GLOBALOPTIONS [INTERNAL] 
    %
    %
    %Collects global CasADi options.
    %
    %Note to developers: 
    %use sparingly. Global options are - in general - a 
    %rather bad idea
    %
    %this class must never be instantiated. Access its static members 
    %directly 
    %
    %Joris Gillis
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_23m
    %
    %C++ includes: global_options.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = GlobalOptions(varargin)
    %GLOBALOPTIONS 
    %
    %  new_obj = GLOBALOPTIONS()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(957, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(958, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = setSimplificationOnTheFly(varargin)
    %SETSIMPLIFICATIONONTHEFLY 
    %
    %  SETSIMPLIFICATIONONTHEFLY(bool flag)
    %
    %
     [varargout{1:nargout}] = casadiMEX(947, varargin{:});
    end
    function varargout = getSimplificationOnTheFly(varargin)
    %GETSIMPLIFICATIONONTHEFLY 
    %
    %  bool = GETSIMPLIFICATIONONTHEFLY()
    %
    %
     [varargout{1:nargout}] = casadiMEX(948, varargin{:});
    end
    function varargout = setHierarchicalSparsity(varargin)
    %SETHIERARCHICALSPARSITY 
    %
    %  SETHIERARCHICALSPARSITY(bool flag)
    %
    %
     [varargout{1:nargout}] = casadiMEX(949, varargin{:});
    end
    function varargout = getHierarchicalSparsity(varargin)
    %GETHIERARCHICALSPARSITY 
    %
    %  bool = GETHIERARCHICALSPARSITY()
    %
    %
     [varargout{1:nargout}] = casadiMEX(950, varargin{:});
    end
    function varargout = setCasadiPath(varargin)
    %SETCASADIPATH 
    %
    %  SETCASADIPATH(char path)
    %
    %
     [varargout{1:nargout}] = casadiMEX(951, varargin{:});
    end
    function varargout = getCasadiPath(varargin)
    %GETCASADIPATH 
    %
    %  char = GETCASADIPATH()
    %
    %
     [varargout{1:nargout}] = casadiMEX(952, varargin{:});
    end
    function varargout = setCasadiIncludePath(varargin)
    %SETCASADIINCLUDEPATH 
    %
    %  SETCASADIINCLUDEPATH(char path)
    %
    %
     [varargout{1:nargout}] = casadiMEX(953, varargin{:});
    end
    function varargout = getCasadiIncludePath(varargin)
    %GETCASADIINCLUDEPATH 
    %
    %  char = GETCASADIINCLUDEPATH()
    %
    %
     [varargout{1:nargout}] = casadiMEX(954, varargin{:});
    end
    function varargout = setMaxNumDir(varargin)
    %SETMAXNUMDIR 
    %
    %  SETMAXNUMDIR(int ndir)
    %
    %
     [varargout{1:nargout}] = casadiMEX(955, varargin{:});
    end
    function varargout = getMaxNumDir(varargin)
    %GETMAXNUMDIR 
    %
    %  int = GETMAXNUMDIR()
    %
    %
     [varargout{1:nargout}] = casadiMEX(956, varargin{:});
    end
  end
end
