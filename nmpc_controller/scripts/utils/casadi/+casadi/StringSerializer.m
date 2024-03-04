classdef  StringSerializer < casadi.SerializerBase
    %STRINGSERIALIZER [INTERNAL] C++ includes: serializer.hpp
    %
    %
    %
    %
  methods
    function self = StringSerializer(varargin)
    %STRINGSERIALIZER [INTERNAL] 
    %
    %  new_obj = STRINGSERIALIZER()
    %
    %Advanced serialization of CasADi objects.
    %
    %This class is intended for advanced users that want to circumvent the 
    %
    %restrictions of standard pickling/matlab save load, ie no raw SX/MX 
    %symbols
    % allowed.
    %
    %
    %
    %::
    %
    %  x = SX.sym('x');
    %  s = StringSerializer();
    %  s.pack(x);
    %  s.pack(sin(x));
    %   
    %  data = s.encode();
    %  
    %  s = StringDeserializer(data);
    %  a = s.unpack();
    %  b = s.unpack();
    %  
    %
    %
    %
    %Note: Saving SX/MX objects individually has a substantial overhead 
    %(both 
    %time and length of encoded string). You are encouraged to use 
    %the 
    %vector/list variants of 'save' for SX/MX to reduce the overhead.
    %
    %See: 
    % Function::save,  Function::serialize,  StringDeserializer,  
    %FileSerializer
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7o
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.hpp#L203
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.cpp#L37-L39
    %
    %
    %
      self@casadi.SerializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1186, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1187, self);
          self.SwigClear();
        end
    end
    function varargout = encode(self,varargin)
    %ENCODE [INTERNAL] 
    %
    %  char = ENCODE(self)
    %
    %Returns a string that holds the serialized objects.
    %
    %As a side effect, this method clears the internal buffer
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.hpp#L211
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.cpp#L85-L90
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1188, self, varargin{:});
    end
  end
  methods(Static)
  end
end
