classdef SwigRef < handle
  properties(Hidden = true, Access = public) 
    swigPtr
  end
  methods(Static = true, Access = protected)
    function obj = Null()
      persistent obj_null
      if isempty(obj_null)
        obj_null = SwigRef();
      end
      obj = obj_null;
    end
  end
  methods
    function out = saveobj(self)
      warning('Serializing SWIG objects not supported.');
      out = saveobj('Serializing SWIG object not supported');
    end
    function b = isnull(self)
      b = isempty(self.swigPtr);
    end
    function SwigSet(self,ptr)
        self.swigPtr = ptr;
    end
    function SwigClear(self)
        self.swigPtr = [];
    end
    function ptr = SwigGet(self)
        ptr = self.swigPtr;
    end
  end
   methods(Static)
    function obj = loadobj(s)
      warning('Serializing SWIG objects not supported.');
      obj = SwigRef.Null();
    end
  end
end
