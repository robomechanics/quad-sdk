classdef  FileSerializer < casadi.SerializerBase
    %FILESERIALIZER [INTERNAL] C++ includes: serializer.hpp
    %
    %
    %
    %
  methods
    function self = FileSerializer(varargin)
    %FILESERIALIZER [INTERNAL] 
    %
    %  new_obj = FILESERIALIZER()
    %
    %Advanced serialization of CasADi objects.
    %
    %See: 
    % StringSerializer,  FileDeserializer
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_7q
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.hpp#L221
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/serializer.cpp#L41-L49
    %
    %
    %
      self@casadi.SerializerBase(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1189, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1190, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
