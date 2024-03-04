classdef  StringDeserializer < casadi.DeserializerBase
    %STRINGDESERIALIZER [INTERNAL] C++ includes: serializer.hpp
    %
    %
    %
    %
  methods
    function self = StringDeserializer(varargin)
    %STRINGDESERIALIZER [INTERNAL] 
    %
    %  new_obj = STRINGDESERIALIZER()
    %
    %Advanced deserialization of CasADi objects.
    %
    %See: 
    % StringDeserializer
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7r
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.hpp#L233
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.cpp#L114-L117
    %
    %
    %
      self@casadi.DeserializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1191, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1192, self);
          self.SwigClear();
        end
    end
    function varargout = decode(self,varargin)
    %DECODE [INTERNAL] 
    %
    %  DECODE(self, char string)
    %
    %Sets the string to deserialize objects from.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7s
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.hpp#L240
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.cpp#L91-L96
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1193, self, varargin{:});
    end
  end
  methods(Static)
  end
end
