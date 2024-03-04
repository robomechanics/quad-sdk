classdef  WeakRef < casadi.SharedObject
    %WEAKREF [INTERNAL] 
    %
    %
    %Weak reference type.
    %
    %A weak reference to a  SharedObject
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ax
    %
    %C++ includes: shared_object.hpp
    %
    %
  methods
    function varargout = shared(self,varargin)
    %SHARED [INTERNAL] 
    %
    %  SharedObject = SHARED(self)
    %
    %Get a shared (owning) reference.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_b0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L198
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L141-L147
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(31, self, varargin{:});
    end
    function varargout = alive(self,varargin)
    %ALIVE [INTERNAL] 
    %
    %  bool = ALIVE(self)
    %
    %Check if alive.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_b1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L203
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L137-L139
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(32, self, varargin{:});
    end
    function self = WeakRef(varargin)
    %WEAKREF 
    %
    %  new_obj = WEAKREF(int dummy)
    %  new_obj = WEAKREF(SharedObject shared)
    %
    %
    %.......
    %
    %::
    %
    %  WEAKREF(int dummy)
    %
    %
    %
    %[INTERNAL] 
    %Default constructor.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ay
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L188
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L133-L135
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
    %  WEAKREF(SharedObject shared)
    %
    %
    %
    %[INTERNAL] 
    %Construct from a shared object (also implicit type conversion)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_az
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L193
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L157-L159
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(33, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(34, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
