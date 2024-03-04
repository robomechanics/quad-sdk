classdef  DeserializerBase < SwigRef
    %DESERIALIZERBASE [INTERNAL] C++ includes: serializer.hpp
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
          casadiMEX(1142, self);
          self.SwigClear();
        end
    end
    function varargout = internal_pop_type(self,varargin)
    %INTERNAL_POP_TYPE [INTERNAL] 
    %
    %  casadi::SerializerBase::SerializationType = INTERNAL_POP_TYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1143, self, varargin{:});
    end
    function varargout = blind_unpack_sparsity(self,varargin)
    %BLIND_UNPACK_SPARSITY [INTERNAL] 
    %
    %  Sparsity = BLIND_UNPACK_SPARSITY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1144, self, varargin{:});
    end
    function varargout = blind_unpack_mx(self,varargin)
    %BLIND_UNPACK_MX [INTERNAL] 
    %
    %  MX = BLIND_UNPACK_MX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1145, self, varargin{:});
    end
    function varargout = blind_unpack_dm(self,varargin)
    %BLIND_UNPACK_DM [INTERNAL] 
    %
    %  DM = BLIND_UNPACK_DM(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1146, self, varargin{:});
    end
    function varargout = blind_unpack_sx(self,varargin)
    %BLIND_UNPACK_SX [INTERNAL] 
    %
    %  SX = BLIND_UNPACK_SX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1147, self, varargin{:});
    end
    function varargout = blind_unpack_linsol(self,varargin)
    %BLIND_UNPACK_LINSOL [INTERNAL] 
    %
    %  Linsol = BLIND_UNPACK_LINSOL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1148, self, varargin{:});
    end
    function varargout = blind_unpack_function(self,varargin)
    %BLIND_UNPACK_FUNCTION [INTERNAL] 
    %
    %  Function = BLIND_UNPACK_FUNCTION(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1149, self, varargin{:});
    end
    function varargout = blind_unpack_generictype(self,varargin)
    %BLIND_UNPACK_GENERICTYPE [INTERNAL] 
    %
    %  GenericType = BLIND_UNPACK_GENERICTYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1150, self, varargin{:});
    end
    function varargout = blind_unpack_int(self,varargin)
    %BLIND_UNPACK_INT [INTERNAL] 
    %
    %  int = BLIND_UNPACK_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1151, self, varargin{:});
    end
    function varargout = blind_unpack_double(self,varargin)
    %BLIND_UNPACK_DOUBLE [INTERNAL] 
    %
    %  double = BLIND_UNPACK_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1152, self, varargin{:});
    end
    function varargout = blind_unpack_string(self,varargin)
    %BLIND_UNPACK_STRING [INTERNAL] 
    %
    %  char = BLIND_UNPACK_STRING(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1153, self, varargin{:});
    end
    function varargout = blind_unpack_sparsity_vector(self,varargin)
    %BLIND_UNPACK_SPARSITY_VECTOR [INTERNAL] 
    %
    %  {Sparsity} = BLIND_UNPACK_SPARSITY_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1154, self, varargin{:});
    end
    function varargout = blind_unpack_mx_vector(self,varargin)
    %BLIND_UNPACK_MX_VECTOR [INTERNAL] 
    %
    %  {MX} = BLIND_UNPACK_MX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1155, self, varargin{:});
    end
    function varargout = blind_unpack_dm_vector(self,varargin)
    %BLIND_UNPACK_DM_VECTOR [INTERNAL] 
    %
    %  {DM} = BLIND_UNPACK_DM_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1156, self, varargin{:});
    end
    function varargout = blind_unpack_sx_vector(self,varargin)
    %BLIND_UNPACK_SX_VECTOR [INTERNAL] 
    %
    %  {SX} = BLIND_UNPACK_SX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1157, self, varargin{:});
    end
    function varargout = blind_unpack_linsol_vector(self,varargin)
    %BLIND_UNPACK_LINSOL_VECTOR [INTERNAL] 
    %
    %  std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > = BLIND_UNPACK_LINSOL_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1158, self, varargin{:});
    end
    function varargout = blind_unpack_function_vector(self,varargin)
    %BLIND_UNPACK_FUNCTION_VECTOR [INTERNAL] 
    %
    %  {Function} = BLIND_UNPACK_FUNCTION_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1159, self, varargin{:});
    end
    function varargout = blind_unpack_generictype_vector(self,varargin)
    %BLIND_UNPACK_GENERICTYPE_VECTOR [INTERNAL] 
    %
    %  {GenericType} = BLIND_UNPACK_GENERICTYPE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1160, self, varargin{:});
    end
    function varargout = blind_unpack_int_vector(self,varargin)
    %BLIND_UNPACK_INT_VECTOR [INTERNAL] 
    %
    %  [int] = BLIND_UNPACK_INT_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1161, self, varargin{:});
    end
    function varargout = blind_unpack_double_vector(self,varargin)
    %BLIND_UNPACK_DOUBLE_VECTOR [INTERNAL] 
    %
    %  [double] = BLIND_UNPACK_DOUBLE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1162, self, varargin{:});
    end
    function varargout = blind_unpack_string_vector(self,varargin)
    %BLIND_UNPACK_STRING_VECTOR [INTERNAL] 
    %
    %  {char} = BLIND_UNPACK_STRING_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1163, self, varargin{:});
    end
    function varargout = unpack_sparsity(self,varargin)
    %UNPACK_SPARSITY [INTERNAL] 
    %
    %  Sparsity = UNPACK_SPARSITY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1164, self, varargin{:});
    end
    function varargout = unpack_mx(self,varargin)
    %UNPACK_MX [INTERNAL] 
    %
    %  MX = UNPACK_MX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1165, self, varargin{:});
    end
    function varargout = unpack_dm(self,varargin)
    %UNPACK_DM [INTERNAL] 
    %
    %  DM = UNPACK_DM(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1166, self, varargin{:});
    end
    function varargout = unpack_sx(self,varargin)
    %UNPACK_SX [INTERNAL] 
    %
    %  SX = UNPACK_SX(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1167, self, varargin{:});
    end
    function varargout = unpack_linsol(self,varargin)
    %UNPACK_LINSOL [INTERNAL] 
    %
    %  Linsol = UNPACK_LINSOL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1168, self, varargin{:});
    end
    function varargout = unpack_function(self,varargin)
    %UNPACK_FUNCTION [INTERNAL] 
    %
    %  Function = UNPACK_FUNCTION(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1169, self, varargin{:});
    end
    function varargout = unpack_generictype(self,varargin)
    %UNPACK_GENERICTYPE [INTERNAL] 
    %
    %  GenericType = UNPACK_GENERICTYPE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1170, self, varargin{:});
    end
    function varargout = unpack_int(self,varargin)
    %UNPACK_INT [INTERNAL] 
    %
    %  int = UNPACK_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1171, self, varargin{:});
    end
    function varargout = unpack_double(self,varargin)
    %UNPACK_DOUBLE [INTERNAL] 
    %
    %  double = UNPACK_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1172, self, varargin{:});
    end
    function varargout = unpack_string(self,varargin)
    %UNPACK_STRING [INTERNAL] 
    %
    %  char = UNPACK_STRING(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1173, self, varargin{:});
    end
    function varargout = unpack_sparsity_vector(self,varargin)
    %UNPACK_SPARSITY_VECTOR [INTERNAL] 
    %
    %  {Sparsity} = UNPACK_SPARSITY_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1174, self, varargin{:});
    end
    function varargout = unpack_mx_vector(self,varargin)
    %UNPACK_MX_VECTOR [INTERNAL] 
    %
    %  {MX} = UNPACK_MX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1175, self, varargin{:});
    end
    function varargout = unpack_dm_vector(self,varargin)
    %UNPACK_DM_VECTOR [INTERNAL] 
    %
    %  {DM} = UNPACK_DM_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1176, self, varargin{:});
    end
    function varargout = unpack_sx_vector(self,varargin)
    %UNPACK_SX_VECTOR [INTERNAL] 
    %
    %  {SX} = UNPACK_SX_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1177, self, varargin{:});
    end
    function varargout = unpack_linsol_vector(self,varargin)
    %UNPACK_LINSOL_VECTOR [INTERNAL] 
    %
    %  std::vector< casadi::Linsol,std::allocator< casadi::Linsol > > = UNPACK_LINSOL_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1178, self, varargin{:});
    end
    function varargout = unpack_function_vector(self,varargin)
    %UNPACK_FUNCTION_VECTOR [INTERNAL] 
    %
    %  {Function} = UNPACK_FUNCTION_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1179, self, varargin{:});
    end
    function varargout = unpack_generictype_vector(self,varargin)
    %UNPACK_GENERICTYPE_VECTOR [INTERNAL] 
    %
    %  {GenericType} = UNPACK_GENERICTYPE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1180, self, varargin{:});
    end
    function varargout = unpack_int_vector(self,varargin)
    %UNPACK_INT_VECTOR [INTERNAL] 
    %
    %  [int] = UNPACK_INT_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1181, self, varargin{:});
    end
    function varargout = unpack_double_vector(self,varargin)
    %UNPACK_DOUBLE_VECTOR [INTERNAL] 
    %
    %  [double] = UNPACK_DOUBLE_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1182, self, varargin{:});
    end
    function varargout = unpack_string_vector(self,varargin)
    %UNPACK_STRING_VECTOR [INTERNAL] 
    %
    %  {char} = UNPACK_STRING_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1183, self, varargin{:});
    end
    function varargout = connect(self,varargin)
    %CONNECT [INTERNAL] 
    %
    %  CONNECT(self, SerializerBase s)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1184, self, varargin{:});
    end
    function varargout = reset(self,varargin)
    %RESET [INTERNAL] 
    %
    %  RESET(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1185, self, varargin{:});
    end

    function out = unpack(self)
      type = casadi.SerializerBase.type_to_string(self.internal_pop_type);
      out = self.(['blind_unpack_' type]);
    end
      function self = DeserializerBase(varargin)
    %DESERIALIZERBASE [INTERNAL] C++ includes: serializer.hpp
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
  end
end
