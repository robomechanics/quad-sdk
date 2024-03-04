classdef  SerializerBase < SwigRef
    %SERIALIZERBASE [INTERNAL] C++ includes: serializer.hpp
    %
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1137, self);
          self.SwigClear();
        end
    end
    function varargout = pack(self,varargin)
    %PACK [INTERNAL] 
    %
    %  PACK(self, Linsol e)
    %  PACK(self, std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > const & e)
    %  PACK(self, int e)
    %  PACK(self, double e)
    %  PACK(self, {Sparsity} e)
    %  PACK(self, Sparsity e)
    %  PACK(self, [double] e)
    %  PACK(self, [int] e)
    %  PACK(self, {char} e)
    %  PACK(self, DM e)
    %  PACK(self, {DM} e)
    %  PACK(self, SX e)
    %  PACK(self, {SX} e)
    %  PACK(self, MX e)
    %  PACK(self, {MX} e)
    %  PACK(self, char e)
    %  PACK(self, {Function} e)
    %  PACK(self, Function e)
    %  PACK(self, {GenericType} e)
    %  PACK(self, GenericType e)
    %
    %
    %.......
    %
    %::
    %
    %  PACK(self, DM e)
    %  PACK(self, {DM} e)
    %  PACK(self, SX e)
    %  PACK(self, {SX} e)
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
    %  PACK(self, Linsol e)
    %  PACK(self, std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > const & e)
    %  PACK(self, int e)
    %  PACK(self, double e)
    %  PACK(self, {Sparsity} e)
    %  PACK(self, Sparsity e)
    %  PACK(self, [double] e)
    %  PACK(self, [int] e)
    %  PACK(self, {char} e)
    %  PACK(self, MX e)
    %  PACK(self, {MX} e)
    %  PACK(self, char e)
    %  PACK(self, {Function} e)
    %  PACK(self, Function e)
    %  PACK(self, {GenericType} e)
    %  PACK(self, GenericType e)
    %
    %
    %
    %[INTERNAL] 
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1138, self, varargin{:});
    end
    function varargout = connect(self,varargin)
    %CONNECT [INTERNAL] 
    %
    %  CONNECT(self, DeserializerBase s)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1140, self, varargin{:});
    end
    function varargout = reset(self,varargin)
    %RESET [INTERNAL] 
    %
    %  RESET(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1141, self, varargin{:});
    end
    function self = SerializerBase(varargin)
    %SERIALIZERBASE [INTERNAL] C++ includes: serializer.hpp
    %
    %
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
    function v = internal_SERIALIZED_SPARSITY()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 125);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_MX()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 126);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DM()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 127);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SX()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 128);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_LINSOL()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 129);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_FUNCTION()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 130);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_GENERICTYPE()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 131);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_INT()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 132);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DOUBLE()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 133);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_STRING()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 134);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SPARSITY_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 135);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_MX_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 136);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DM_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 137);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_SX_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 138);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_LINSOL_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 139);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_FUNCTION_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 140);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_GENERICTYPE_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 141);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_INT_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 142);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_DOUBLE_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 143);
      end
      v = vInitialized;
    end
    function v = internal_SERIALIZED_STRING_VECTOR()
      persistent vInitialized;
      if isempty(vInitialized)
        vInitialized = casadiMEX(0, 144);
      end
      v = vInitialized;
    end
    function varargout = type_to_string(varargin)
    %TYPE_TO_STRING 
    %
    %  char = TYPE_TO_STRING(casadi::SerializerBase::SerializationType type)
    %
    %
     [varargout{1:nargout}] = casadiMEX(1139, varargin{:});
    end
  end
end
