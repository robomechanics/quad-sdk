classdef  OptiCallback < SwigRef
    %OPTICALLBACK [INTERNAL] C++ includes: optistack.hpp
    %
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = OptiCallback(varargin)
    %OPTICALLBACK [INTERNAL] 
    %
    %  new_obj = OPTICALLBACK(self)
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        if strcmp(class(self),'director_basic.OptiCallback')
          tmp = casadiMEX(1260, 0, varargin{:});
        else
          tmp = casadiMEX(1260, self, varargin{:});
        end
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = call(self,varargin)
    %CALL [INTERNAL] 
    %
    %  CALL(self, int i)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1261, self, varargin{:});
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1262, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
