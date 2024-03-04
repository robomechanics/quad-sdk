classdef  DM < casadi.MatrixCommon & casadi.GenericExpressionCommon & casadi.GenDM & casadi.PrintableCommon
    %DM 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = has_nz(self,varargin)
    %HAS_NZ [INTERNAL] 
    %
    %  bool = HAS_NZ(self, int rr, int cc)
    %
    %Returns true if the matrix has a non-zero at location rr, cc.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L219
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L65-L67
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(442, self, varargin{:});
    end
    function varargout = nonzero(self,varargin)
    %NONZERO [INTERNAL] 
    %
    %  bool = NONZERO(self)
    %
    %Returns the truth value of a  Matrix.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L222
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L70-L76
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(443, self, varargin{:});
    end
    function varargout = get(self,varargin)
    %GET 
    %
    %  DM = GET(self, bool ind1, Sparsity sp)
    %  DM = GET(self, bool ind1, Slice rr)
    %  DM = GET(self, bool ind1, IM rr)
    %  DM = GET(self, bool ind1, Slice rr, Slice cc)
    %  DM = GET(self, bool ind1, Slice rr, IM cc)
    %  DM = GET(self, bool ind1, IM rr, Slice cc)
    %  DM = GET(self, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(444, self, varargin{:});
    end
    function varargout = set(self,varargin)
    %SET 
    %
    %  SET(self, DM m, bool ind1, Sparsity sp)
    %  SET(self, DM m, bool ind1, Slice rr)
    %  SET(self, DM m, bool ind1, IM rr)
    %  SET(self, DM m, bool ind1, Slice rr, Slice cc)
    %  SET(self, DM m, bool ind1, Slice rr, IM cc)
    %  SET(self, DM m, bool ind1, IM rr, Slice cc)
    %  SET(self, DM m, bool ind1, IM rr, IM cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(445, self, varargin{:});
    end
    function varargout = get_nz(self,varargin)
    %GET_NZ 
    %
    %  DM = GET_NZ(self, bool ind1, Slice k)
    %  DM = GET_NZ(self, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(446, self, varargin{:});
    end
    function varargout = set_nz(self,varargin)
    %SET_NZ 
    %
    %  SET_NZ(self, DM m, bool ind1, Slice k)
    %  SET_NZ(self, DM m, bool ind1, IM k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(447, self, varargin{:});
    end
    function varargout = uplus(self,varargin)
    %UPLUS 
    %
    %  DM = UPLUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(448, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
    %UMINUS 
    %
    %  DM = UMINUS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(449, self, varargin{:});
    end
    function varargout = printme(self,varargin)
    %PRINTME 
    %
    %  DM = PRINTME(self, DM y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(455, self, varargin{:});
    end
    function varargout = T(self,varargin)
    %T [INTERNAL] 
    %
    %  DM = T(self)
    %
    %Transpose the matrix.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L494
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(456, self, varargin{:});
    end
    function varargout = print_split(self,varargin)
    %PRINT_SPLIT [INTERNAL] 
    %
    %  [{char} OUTPUT, {char} OUTPUT] = PRINT_SPLIT(self)
    %
    %Get strings corresponding to the nonzeros and the 
    %interdependencies.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L873
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L696-L700
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(462, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP [INTERNAL] 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %Print a representation of the object.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L877
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(463, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR [INTERNAL] 
    %
    %  char = STR(self, bool more)
    %
    %Get string representation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L880
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(464, self, varargin{:});
    end
    function varargout = print_scalar(self,varargin)
    %PRINT_SCALAR [INTERNAL] 
    %
    %  std::ostream & = PRINT_SCALAR(self)
    %
    %Print scalar.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L883
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L609-L633
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(465, self, varargin{:});
    end
    function varargout = print_vector(self,varargin)
    %PRINT_VECTOR [INTERNAL] 
    %
    %  std::ostream & = PRINT_VECTOR(self, bool truncate)
    %
    %Print vector-style.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L886
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L636-L638
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(466, self, varargin{:});
    end
    function varargout = print_dense(self,varargin)
    %PRINT_DENSE [INTERNAL] 
    %
    %  std::ostream & = PRINT_DENSE(self, bool truncate)
    %
    %Print dense matrix-stype.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L889
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L686-L688
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(467, self, varargin{:});
    end
    function varargout = print_sparse(self,varargin)
    %PRINT_SPARSE [INTERNAL] 
    %
    %  std::ostream & = PRINT_SPARSE(self, bool truncate)
    %
    %Print sparse matrix style.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L892
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_impl.hpp#L691-L693
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(468, self, varargin{:});
    end
    function varargout = clear(self,varargin)
    %CLEAR [INTERNAL] 
    %
    %  CLEAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(469, self, varargin{:});
    end
    function varargout = resize(self,varargin)
    %RESIZE [INTERNAL] 
    %
    %  RESIZE(self, int nrow, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(470, self, varargin{:});
    end
    function varargout = reserve(self,varargin)
    %RESERVE [INTERNAL] 
    %
    %  RESERVE(self, int nnz)
    %  RESERVE(self, int nnz, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(471, self, varargin{:});
    end
    function varargout = erase(self,varargin)
    %ERASE [INTERNAL] 
    %
    %  ERASE(self, [int] rr, bool ind1)
    %  ERASE(self, [int] rr, [int] cc, bool ind1)
    %
    %Erase a submatrix (leaving structural zeros in its place)
    %
    %Erase elements of a matrix
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L937
    %
    %
    %
    %.......
    %
    %::
    %
    %  ERASE(self, [int] rr, bool ind1)
    %
    %
    %
    %[INTERNAL] 
    %Erase a submatrix (leaving structural zeros in its place)
    %
    %Erase elements of a matrix
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19g
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L937
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
    %  ERASE(self, [int] rr, [int] cc, bool ind1)
    %
    %
    %
    %[INTERNAL] 
    %Erase a submatrix (leaving structural zeros in its place)
    %
    %Erase rows and/or columns of a matrix
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19f
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L929
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(472, self, varargin{:});
    end
    function varargout = remove(self,varargin)
    %REMOVE [INTERNAL] 
    %
    %  REMOVE(self, [int] rr, [int] cc)
    %
    %Remove columns and rows.
    %
    %Remove/delete rows and/or columns of a matrix
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L944
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(473, self, varargin{:});
    end
    function varargout = enlarge(self,varargin)
    %ENLARGE [INTERNAL] 
    %
    %  ENLARGE(self, int nrow, int ncol, [int] rr, [int] cc, bool ind1)
    %
    %Enlarge matrix.
    %
    %Make the matrix larger by inserting empty rows and columns, keeping 
    %the 
    %existing non-zeros
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19i
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L952
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(474, self, varargin{:});
    end
    function varargout = sparsity(self,varargin)
    %SPARSITY [INTERNAL] 
    %
    %  Sparsity = SPARSITY(self)
    %
    %Get an owning reference to the sparsity pattern.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L979
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(475, self, varargin{:});
    end
    function varargout = element_hash(self,varargin)
    %ELEMENT_HASH 
    %
    %  int = ELEMENT_HASH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(480, self, varargin{:});
    end
    function varargout = is_regular(self,varargin)
    %IS_REGULAR 
    %
    %  bool = IS_REGULAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(481, self, varargin{:});
    end
    function varargout = is_smooth(self,varargin)
    %IS_SMOOTH 
    %
    %  bool = IS_SMOOTH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(482, self, varargin{:});
    end
    function varargout = is_leaf(self,varargin)
    %IS_LEAF 
    %
    %  bool = IS_LEAF(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(483, self, varargin{:});
    end
    function varargout = is_commutative(self,varargin)
    %IS_COMMUTATIVE 
    %
    %  bool = IS_COMMUTATIVE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(484, self, varargin{:});
    end
    function varargout = is_symbolic(self,varargin)
    %IS_SYMBOLIC 
    %
    %  bool = IS_SYMBOLIC(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(485, self, varargin{:});
    end
    function varargout = is_valid_input(self,varargin)
    %IS_VALID_INPUT 
    %
    %  bool = IS_VALID_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(486, self, varargin{:});
    end
    function varargout = has_duplicates(self,varargin)
    %HAS_DUPLICATES 
    %
    %  bool = HAS_DUPLICATES(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(487, self, varargin{:});
    end
    function varargout = reset_input(self,varargin)
    %RESET_INPUT 
    %
    %  RESET_INPUT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(488, self, varargin{:});
    end
    function varargout = is_constant(self,varargin)
    %IS_CONSTANT [INTERNAL] 
    %
    %  bool = IS_CONSTANT(self)
    %
    %Check if the matrix is constant (note that false negative 
    %answers are 
    %possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19v
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1085
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(489, self, varargin{:});
    end
    function varargout = is_integer(self,varargin)
    %IS_INTEGER [INTERNAL] 
    %
    %  bool = IS_INTEGER(self)
    %
    %Check if the matrix is integer-valued.
    %
    %(note that false negative answers are possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19w
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1092
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(490, self, varargin{:});
    end
    function varargout = is_zero(self,varargin)
    %IS_ZERO [INTERNAL] 
    %
    %  bool = IS_ZERO(self)
    %
    %check if the matrix is 0 (note that false negative answers are 
    %
    %possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19x
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1097
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(491, self, varargin{:});
    end
    function varargout = is_one(self,varargin)
    %IS_ONE [INTERNAL] 
    %
    %  bool = IS_ONE(self)
    %
    %check if the matrix is 1 (note that false negative answers are 
    %
    %possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19y
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1102
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(492, self, varargin{:});
    end
    function varargout = is_minus_one(self,varargin)
    %IS_MINUS_ONE [INTERNAL] 
    %
    %  bool = IS_MINUS_ONE(self)
    %
    %check if the matrix is -1 (note that false negative answers are
    % 
    %possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19z
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1107
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(493, self, varargin{:});
    end
    function varargout = is_eye(self,varargin)
    %IS_EYE [INTERNAL] 
    %
    %  bool = IS_EYE(self)
    %
    %check if the matrix is an identity matrix (note that false 
    %negative 
    %answers
    %
    %are possible)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1a0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1114
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(494, self, varargin{:});
    end
    function varargout = op(self,varargin)
    %OP 
    %
    %  int = OP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(495, self, varargin{:});
    end
    function varargout = is_op(self,varargin)
    %IS_OP 
    %
    %  bool = IS_OP(self, int op)
    %
    %
      [varargout{1:nargout}] = casadiMEX(496, self, varargin{:});
    end
    function varargout = has_zeros(self,varargin)
    %HAS_ZEROS [INTERNAL] 
    %
    %  bool = HAS_ZEROS(self)
    %
    %Check if the matrix has any zero entries which are not 
    %structural 
    %zeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1a1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1125
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(497, self, varargin{:});
    end
    function varargout = nonzeros(self,varargin)
    %NONZEROS [INTERNAL] 
    %
    %  [double] = NONZEROS(self)
    %
    %Get all nonzeros.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1a4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1142
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(498, self, varargin{:});
    end
    function varargout = elements(self,varargin)
    %ELEMENTS [INTERNAL] 
    %
    %  [double] = ELEMENTS(self)
    %
    %Get all elements.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1a3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1135
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(499, self, varargin{:});
    end
    function varargout = to_double(self,varargin)
    %TO_DOUBLE 
    %
    %  double = TO_DOUBLE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(500, self, varargin{:});
    end
    function varargout = to_int(self,varargin)
    %TO_INT 
    %
    %  int = TO_INT(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(501, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(502, self, varargin{:});
    end
    function varargout = dep(self,varargin)
    %DEP 
    %
    %  DM = DEP(self, int ch)
    %
    %
      [varargout{1:nargout}] = casadiMEX(503, self, varargin{:});
    end
    function varargout = n_dep(self,varargin)
    %N_DEP 
    %
    %  int = N_DEP(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(504, self, varargin{:});
    end
    function varargout = export_code(self,varargin)
    %EXPORT_CODE 
    %
    %  std::ostream & = EXPORT_CODE(self, char lang, struct options)
    %
    %
      [varargout{1:nargout}] = casadiMEX(510, self, varargin{:});
    end
    function varargout = info(self,varargin)
    %INFO 
    %
    %  struct = INFO(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(511, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE [INTERNAL] 
    %
    %  char = SERIALIZE(self)
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %Serialize an object.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ah
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1249
    %
    %
    %
    %.......
    %
    %::
    %
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %
    %
    %[INTERNAL] 
    %Serialize an object.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ah
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1249
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
    %  SERIALIZE(self)
    %
    %
    %
    %[INTERNAL] 
    %Serialize.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ae
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1234
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(512, self, varargin{:});
    end
    function varargout = to_file(self,varargin)
    %TO_FILE [INTERNAL] 
    %
    %  TO_FILE(self, char filename, char format)
    %
    %Export numerical matrix to file
    %
    %Supported formats:
    %
    %
    %
    %::
    %
    %  *   - .mtx   Matrix Market (sparse)
    %  *   - .txt   Ascii full precision representation (sparse)
    %  *            Whitespace separated, aligned.
    %  *            Comments with # % or /
    %  *            Uses C locale
    %  *            Structural zeros represented by 00
    %  *            Does not scale well for large sparse matrices
    %  * 
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1269
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(514, self, varargin{:});
    end
    function self = DM(varargin)
    %DM 
    %
    %  new_obj = DM()
    %  new_obj = DM(Sparsity sp)
    %  new_obj = DM(double val)
    %  new_obj = DM(DM m)
    %  new_obj = DM(int nrow, int ncol)
    %  new_obj = DM(Sparsity sp, DM d)
    %
    %
      self@casadi.MatrixCommon(SwigRef.Null);
      self@casadi.GenericExpressionCommon(SwigRef.Null);
      self@casadi.GenDM(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(516, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = assign(self,varargin)
    %ASSIGN 
    %
    %  ASSIGN(self, DM rhs)
    %
    %
      [varargout{1:nargout}] = casadiMEX(517, self, varargin{:});
    end
    function varargout = paren(self,varargin)
    %PAREN 
    %
    %  DM = PAREN(self, Sparsity sp)
    %  DM = PAREN(self, IM rr)
    %  DM = PAREN(self, char rr)
    %  DM = PAREN(self, IM rr, IM cc)
    %  DM = PAREN(self, IM rr, char cc)
    %  DM = PAREN(self, char rr, IM cc)
    %  DM = PAREN(self, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(518, self, varargin{:});
    end
    function varargout = paren_asgn(self,varargin)
    %PAREN_ASGN 
    %
    %  PAREN_ASGN(self, DM m, Sparsity sp)
    %  PAREN_ASGN(self, DM m, IM rr)
    %  PAREN_ASGN(self, DM m, char rr)
    %  PAREN_ASGN(self, DM m, IM rr, IM cc)
    %  PAREN_ASGN(self, DM m, IM rr, char cc)
    %  PAREN_ASGN(self, DM m, char rr, IM cc)
    %  PAREN_ASGN(self, DM m, char rr, char cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(519, self, varargin{:});
    end
    function varargout = brace(self,varargin)
    %BRACE 
    %
    %  DM = BRACE(self, IM rr)
    %  DM = BRACE(self, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(520, self, varargin{:});
    end
    function varargout = setbrace(self,varargin)
    %SETBRACE 
    %
    %  SETBRACE(self, DM m, IM rr)
    %  SETBRACE(self, DM m, char rr)
    %
    %
      [varargout{1:nargout}] = casadiMEX(521, self, varargin{:});
    end
    function varargout = end(self,varargin)
    %END 
    %
    %  int = END(self, int i, int n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(522, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL [INTERNAL] 
    %
    %  int = NUMEL(self)
    %  int = NUMEL(self, int k)
    %  int = NUMEL(self, [int] k)
    %  int = NUMEL(self, char rr)
    %
    %Get the number of elements.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ar
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L104
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L1139-L1141
    %
    %
    %
    %.......
    %
    %::
    %
    %  NUMEL(self, int k)
    %  NUMEL(self, [int] k)
    %  NUMEL(self, char rr)
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
    %  NUMEL(self)
    %
    %
    %
    %[INTERNAL] 
    %Get the number of elements.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ar
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L104
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_matrix.hpp#L1139-L1141
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(523, self, varargin{:});
    end
    function varargout = ctranspose(self,varargin)
    %CTRANSPOSE 
    %
    %  DM = CTRANSPOSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(524, self, varargin{:});
    end
    function varargout = full(self,varargin)
    %FULL 
    %
    %  mxArray * = FULL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(525, self, varargin{:});
    end
    function varargout = sparse(self,varargin)
    %SPARSE 
    %
    %  mxArray * = SPARSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(526, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi DM failed:' getReport(exception) ]);
            s = struct;
        end
     end
      function delete(self)
        if self.swigPtr
          casadiMEX(527, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = binary(varargin)
    %BINARY 
    %
    %  DM = BINARY(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(450, varargin{:});
    end
    function varargout = unary(varargin)
    %UNARY 
    %
    %  DM = UNARY(int op, DM x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(451, varargin{:});
    end
    function varargout = scalar_matrix(varargin)
    %SCALAR_MATRIX 
    %
    %  DM = SCALAR_MATRIX(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(452, varargin{:});
    end
    function varargout = matrix_scalar(varargin)
    %MATRIX_SCALAR 
    %
    %  DM = MATRIX_SCALAR(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(453, varargin{:});
    end
    function varargout = matrix_matrix(varargin)
    %MATRIX_MATRIX 
    %
    %  DM = MATRIX_MATRIX(int op, DM x, DM y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(454, varargin{:});
    end
    function varargout = set_max_depth(varargin)
    %SET_MAX_DEPTH 
    %
    %  SET_MAX_DEPTH(int eq_depth)
    %
    %
     [varargout{1:nargout}] = casadiMEX(457, varargin{:});
    end
    function varargout = get_max_depth(varargin)
    %GET_MAX_DEPTH 
    %
    %  int = GET_MAX_DEPTH()
    %
    %
     [varargout{1:nargout}] = casadiMEX(458, varargin{:});
    end
    function varargout = get_input(varargin)
    %GET_INPUT 
    %
    %  {DM} = GET_INPUT(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(459, varargin{:});
    end
    function varargout = get_free(varargin)
    %GET_FREE 
    %
    %  {DM} = GET_FREE(Function f)
    %
    %
     [varargout{1:nargout}] = casadiMEX(460, varargin{:});
    end
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(461, varargin{:});
    end
    function varargout = triplet(varargin)
    %TRIPLET 
    %
    %  DM = TRIPLET([int] row, [int] col, DM d)
    %  DM = TRIPLET([int] row, [int] col, DM d, [int,int] rc)
    %  DM = TRIPLET([int] row, [int] col, DM d, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(476, varargin{:});
    end
    function varargout = inf(varargin)
    %INF [INTERNAL] 
    %
    %  DM = INF(int nrow, int ncol)
    %  DM = INF([int,int] rc)
    %  DM = INF(Sparsity sp)
    %
    %create a matrix with all inf
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1005
    %
    %
    %
    %.......
    %
    %::
    %
    %  INF(int nrow, int ncol)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all inf
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1004
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
    %  INF([int,int] rc)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all inf
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1005
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
    %  INF(Sparsity sp)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all inf
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1003
    %
    %
    %
    %.............
    %
    %
     [varargout{1:nargout}] = casadiMEX(477, varargin{:});
    end
    function varargout = nan(varargin)
    %NAN [INTERNAL] 
    %
    %  DM = NAN(int nrow, int ncol)
    %  DM = NAN([int,int] rc)
    %  DM = NAN(Sparsity sp)
    %
    %create a matrix with all nan
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1014
    %
    %
    %
    %.......
    %
    %::
    %
    %  NAN(int nrow, int ncol)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all nan
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1013
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
    %  NAN([int,int] rc)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all nan
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1014
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
    %  NAN(Sparsity sp)
    %
    %
    %
    %[INTERNAL] 
    %create a matrix with all nan
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_19l
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1012
    %
    %
    %
    %.............
    %
    %
     [varargout{1:nargout}] = casadiMEX(478, varargin{:});
    end
    function varargout = eye(varargin)
    %EYE 
    %
    %  DM = EYE(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(479, varargin{:});
    end
    function varargout = set_precision(varargin)
    %SET_PRECISION 
    %
    %  SET_PRECISION(int precision)
    %
    %
     [varargout{1:nargout}] = casadiMEX(505, varargin{:});
    end
    function varargout = set_width(varargin)
    %SET_WIDTH 
    %
    %  SET_WIDTH(int width)
    %
    %
     [varargout{1:nargout}] = casadiMEX(506, varargin{:});
    end
    function varargout = set_scientific(varargin)
    %SET_SCIENTIFIC 
    %
    %  SET_SCIENTIFIC(bool scientific)
    %
    %
     [varargout{1:nargout}] = casadiMEX(507, varargin{:});
    end
    function varargout = rng(varargin)
    %RNG 
    %
    %  RNG(int seed)
    %
    %
     [varargout{1:nargout}] = casadiMEX(508, varargin{:});
    end
    function varargout = rand(varargin)
    %RAND [INTERNAL] 
    %
    %  DM = RAND(int nrow, int ncol)
    %  DM = RAND([int,int] rc)
    %  DM = RAND(Sparsity sp)
    %
    %Create a matrix with uniformly distributed random numbers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ab
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1201
    %
    %
    %
    %.......
    %
    %::
    %
    %  RAND(Sparsity sp)
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
    %  RAND(int nrow, int ncol)
    %
    %
    %
    %[INTERNAL] 
    %Create a matrix with uniformly distributed random numbers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ab
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1197
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
    %  RAND([int,int] rc)
    %
    %
    %
    %[INTERNAL] 
    %Create a matrix with uniformly distributed random numbers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ab
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/matrix_decl.hpp#L1201
    %
    %
    %
    %.............
    %
    %
     [varargout{1:nargout}] = casadiMEX(509, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  DM = DESERIALIZE(std::istream & stream)
    %  DM = DESERIALIZE(casadi::DeserializingStream & s)
    %  DM = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(513, varargin{:});
    end
    function varargout = from_file(varargin)
    %FROM_FILE 
    %
    %  DM = FROM_FILE(char filename, char format_hint)
    %
    %
     [varargout{1:nargout}] = casadiMEX(515, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.DM.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi DM failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
