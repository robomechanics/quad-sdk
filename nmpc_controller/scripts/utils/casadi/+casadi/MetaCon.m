classdef  MetaCon < casadi.IndexAbstraction
    %METACON 
    %
    %   = METACON()
    %
    %
  methods
    function v = original(self)
      v = casadiMEX(1238, self);
    end
    function v = canon(self)
      v = casadiMEX(1239, self);
    end
    function v = type(self)
      v = casadiMEX(1240, self);
    end
    function v = lb(self)
      v = casadiMEX(1241, self);
    end
    function v = ub(self)
      v = casadiMEX(1242, self);
    end
    function v = n(self)
      v = casadiMEX(1243, self);
    end
    function v = flipped(self)
      v = casadiMEX(1244, self);
    end
    function v = dual_canon(self)
      v = casadiMEX(1245, self);
    end
    function v = dual(self)
      v = casadiMEX(1246, self);
    end
    function v = extra(self)
      v = casadiMEX(1247, self);
    end
    function self = MetaCon(varargin)
    %METACON 
    %
    %  new_obj = METACON()
    %
    %
      self@casadi.IndexAbstraction(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1248, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1249, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
