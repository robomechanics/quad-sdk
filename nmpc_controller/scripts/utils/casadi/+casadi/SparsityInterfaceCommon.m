classdef  SparsityInterfaceCommon < SwigRef
    %SPARSITYINTERFACECOMMON [INTERNAL] 
    %
    %   = SPARSITYINTERFACECOMMON()
    %
    %Sparsity interface class.
    %
    %This is a common base class for  GenericMatrix (i.e.  MX and Matrix<>) and 
    %Sparsity, introducing a uniform syntax and 
    %implementing common 
    %functionality using the curiously recurring 
    %template pattern (CRTP) idiom.
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3d
    %
    %C++ includes: sparsity_interface.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = horzcat(varargin)
    %HORZCAT Concatenate horizontally, six matrices.
    %
    %  Sparsity = HORZCAT({Sparsity} v)
    %  DM = HORZCAT({DM} v)
    %  SX = HORZCAT({SX} v)
    %  MX = HORZCAT({MX} v)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_4e
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L502
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L502-L505
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(45, varargin);
    end
    function varargout = vertcat(varargin)
    %VERTCAT Concatenate vertically, six matrices.
    %
    %  Sparsity = VERTCAT({Sparsity} v)
    %  DM = VERTCAT({DM} v)
    %  SX = VERTCAT({SX} v)
    %  MX = VERTCAT({MX} v)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_4j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L540
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L540-L543
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(46, varargin);
    end
    function varargout = horzsplit(varargin)
    %HORZSPLIT split horizontally, retaining fixed-sized groups of columns
    %
    %  {Sparsity} = HORZSPLIT(Sparsity v, int incr)
    %  {DM} = HORZSPLIT(DM v, int incr)
    %  {SX} = HORZSPLIT(SX v, int incr)
    %  {MX} = HORZSPLIT(MX v, int incr)
    %  {Sparsity} = HORZSPLIT(Sparsity v, [int] offset)
    %  {DM} = HORZSPLIT(DM v, [int] offset)
    %  {SX} = HORZSPLIT(SX v, [int] offset)
    %  {MX} = HORZSPLIT(MX v, [int] offset)
    %
    %
    %Parameters:
    %-----------
    %
    %incr: 
    %Size (width) of each group of columns
    %
    %horzcat(horzsplit(x, ...)) = x
    %
    %\\seealso horzsplit_n
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3h
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L134
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L134-L136
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(47, varargin{:});
    end
    function varargout = horzsplit_n(varargin)
    %HORZSPLIT_N split horizontally, retaining fixed-sized groups of columns
    %
    %  {Sparsity} = HORZSPLIT_N(Sparsity v, int n)
    %  {DM} = HORZSPLIT_N(DM v, int n)
    %  {SX} = HORZSPLIT_N(SX v, int n)
    %  {MX} = HORZSPLIT_N(MX v, int n)
    %
    %
    %Parameters:
    %-----------
    %
    %n: 
    %Number of groups of columns
    %
    %Will error when the number of columns is not a multiple of n
    %
    %horzcat(horzsplit(x, ...)) = x
    %
    %\\seealso horzsplit
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_277
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L149
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L149-L151
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(48, varargin{:});
    end
    function varargout = offset(varargin)
    %OFFSET Helper function, get offsets corresponding to a vector of matrices.
    %
    %  [int] = OFFSET({Sparsity} v, bool vert)
    %  [int] = OFFSET({DM} v, bool vert)
    %  [int] = OFFSET({SX} v, bool vert)
    %  [int] = OFFSET({MX} v, bool vert)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3j
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L169
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L169-L171
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(49, varargin{:});
    end
    function varargout = vertsplit(varargin)
    %VERTSPLIT split vertically, retaining fixed-sized groups of rows
    %
    %  {Sparsity} = VERTSPLIT(Sparsity v, int incr)
    %  {DM} = VERTSPLIT(DM v, int incr)
    %  {SX} = VERTSPLIT(SX v, int incr)
    %  {MX} = VERTSPLIT(MX v, int incr)
    %  {Sparsity} = VERTSPLIT(Sparsity v, [int] offset)
    %  {DM} = VERTSPLIT(DM v, [int] offset)
    %  {SX} = VERTSPLIT(SX v, [int] offset)
    %  {MX} = VERTSPLIT(MX v, [int] offset)
    %
    %
    %Parameters:
    %-----------
    %
    %incr: 
    %Size of each group of rows
    %
    %vertcat(vertsplit(x, ...)) = x
    %
    %
    %
    %::
    %
    %  >>> print vertsplit(SX.sym("a",4))
    %  [SX(a_0), SX(a_1), SX(a_2), SX(a_3)]
    %  
    %
    %
    %
    %
    %
    %::
    %
    %  >>> print vertsplit(SX.sym("a",4),2)
    %  [SX([a_0, a_1]), SX([a_2, a_3])]
    %  
    %
    %
    %
    %If the number of rows is not a multiple of  incr, the last entry returned 
    %will have a size smaller than  incr.
    %
    %
    %
    %::
    %
    %  >>> print vertsplit(DM([0,1,2,3,4]),2)
    %  [DM([0, 1]), DM([2, 3]), DM(4)]
    %  
    %
    %
    %
    %\\seealso vertsplit_n
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3k
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L204
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L204-L206
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(50, varargin{:});
    end
    function varargout = vertsplit_n(varargin)
    %VERTSPLIT_N split vertically, retaining fixed-sized groups of rows
    %
    %  {Sparsity} = VERTSPLIT_N(Sparsity v, int n)
    %  {DM} = VERTSPLIT_N(DM v, int n)
    %  {SX} = VERTSPLIT_N(SX v, int n)
    %  {MX} = VERTSPLIT_N(MX v, int n)
    %
    %
    %Parameters:
    %-----------
    %
    %n: 
    %Number of groups of rows
    %
    %Will error when the number of rows is not a multiple of n
    %
    %vertcat(vertsplit(x, ...)) = x
    %
    %\\seealso vertsplit
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_278
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L219
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L219-L221
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(51, varargin{:});
    end
    function varargout = blockcat(varargin)
    %BLOCKCAT Construct a matrix from 4 blocks.
    %
    %  Sparsity = BLOCKCAT(Sparsity A, Sparsity B, Sparsity C, Sparsity D)
    %  DM = BLOCKCAT(DM A, DM B, DM C, DM D)
    %  SX = BLOCKCAT(SX A, SX B, SX C, SX D)
    %  MX = BLOCKCAT(MX A, MX B, MX C, MX D)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3m
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L234
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L234-L236
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(52, varargin{:});
    end
    function varargout = blocksplit(varargin)
    %BLOCKSPLIT chop up into blocks
    %
    %  {{Sparsity}} = BLOCKSPLIT(Sparsity x, int vert_incr, int horz_incr)
    %  {{DM}} = BLOCKSPLIT(DM x, int vert_incr, int horz_incr)
    %  {{SX}} = BLOCKSPLIT(SX x, int vert_incr, int horz_incr)
    %  {{MX}} = BLOCKSPLIT(MX x, int vert_incr, int horz_incr)
    %  {{Sparsity}} = BLOCKSPLIT(Sparsity x, [int] vert_offset, [int] horz_offset)
    %  {{DM}} = BLOCKSPLIT(DM x, [int] vert_offset, [int] horz_offset)
    %  {{SX}} = BLOCKSPLIT(SX x, [int] vert_offset, [int] horz_offset)
    %  {{MX}} = BLOCKSPLIT(MX x, [int] vert_offset, [int] horz_offset)
    %
    %
    %Parameters:
    %-----------
    %
    %vert_incr: 
    %Defines the increment for block boundaries in row dimension
    %
    %horz_incr: 
    %Defines the increment for block boundaries in column dimension
    %
    %blockcat(blocksplit(x,..., ...)) = x
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3o
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L262
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L262-L264
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(53, varargin{:});
    end
    function varargout = diagcat(varargin)
    %DIAGCAT Concatenate along diagonal, six matrices.
    %
    %  Sparsity = DIAGCAT({Sparsity} A)
    %  DM = DIAGCAT({DM} A)
    %  SX = DIAGCAT({SX} A)
    %  MX = DIAGCAT({MX} A)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_4o
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L578
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L578-L581
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(54, varargin);
    end
    function varargout = diagsplit(varargin)
    %DIAGSPLIT split diagonally, retaining fixed-sized matrices
    %
    %  {Sparsity} = DIAGSPLIT(Sparsity x, int incr)
    %  {DM} = DIAGSPLIT(DM x, int incr)
    %  {SX} = DIAGSPLIT(SX x, int incr)
    %  {MX} = DIAGSPLIT(MX x, int incr)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, [int] output_offset)
    %  {DM} = DIAGSPLIT(DM x, [int] output_offset)
    %  {SX} = DIAGSPLIT(SX x, [int] output_offset)
    %  {MX} = DIAGSPLIT(MX x, [int] output_offset)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, int incr1, int incr2)
    %  {Sparsity} = DIAGSPLIT(Sparsity x, [int] output_offset1, [int] output_offset2)
    %  {DM} = DIAGSPLIT(DM x, int incr1, int incr2)
    %  {DM} = DIAGSPLIT(DM x, [int] output_offset1, [int] output_offset2)
    %  {SX} = DIAGSPLIT(SX x, int incr1, int incr2)
    %  {SX} = DIAGSPLIT(SX x, [int] output_offset1, [int] output_offset2)
    %  {MX} = DIAGSPLIT(MX x, int incr1, int incr2)
    %  {MX} = DIAGSPLIT(MX x, [int] output_offset1, [int] output_offset2)
    %
    %
    %Parameters:
    %-----------
    %
    %incr1: 
    %Row dimension of each matrix
    %
    %incr2: 
    %Column dimension of each matrix
    %
    %diagsplit(diagsplit(x, ...)) = x
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3t
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L324
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L324-L326
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(55, varargin{:});
    end
    function varargout = veccat(varargin)
    %VECCAT concatenate vertically while vectorizing all arguments with vec
    %
    %  Sparsity = VECCAT({Sparsity} x)
    %  DM = VECCAT({DM} x)
    %  SX = VECCAT({SX} x)
    %  MX = VECCAT({MX} x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3u
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L331
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L331-L333
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(56, varargin);
    end
    function varargout = mtimes(varargin)
    %MTIMES Matrix product of n matrices.
    %
    %  Sparsity = MTIMES({Sparsity} args)
    %  DM = MTIMES({DM} args)
    %  SX = MTIMES({SX} args)
    %  MX = MTIMES({MX} args)
    %  Sparsity = MTIMES(Sparsity x, Sparsity y)
    %  DM = MTIMES(DM x, DM y)
    %  SX = MTIMES(SX x, SX y)
    %  MX = MTIMES(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3w
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L345
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L345-L347
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(57, varargin{:});
    end
    function varargout = mac(varargin)
    %MAC Multiply-accumulate operation.
    %
    %  Sparsity = MAC(Sparsity X, Sparsity Y, Sparsity Z)
    %  DM = MAC(DM X, DM Y, DM Z)
    %  SX = MAC(SX X, SX Y, SX Z)
    %  MX = MAC(MX X, MX Y, MX Z)
    %
    %
    %Matrix product of two matrices (x and y), adding the result to a third 
    %
    %matrix z. The result has the same sparsity pattern as C meaning that 
    %other 
    %entries of (x*y) are ignored. The operation is equivalent to: 
    %
    %z+mtimes(x,y).project(z.sparsity()).
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3x
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L358
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L358-L360
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(58, varargin{:});
    end
    function varargout = transpose(varargin)
    %TRANSPOSE Transpose.
    %
    %  Sparsity = TRANSPOSE(Sparsity X)
    %  DM = TRANSPOSE(DM X)
    %  SX = TRANSPOSE(SX X)
    %  MX = TRANSPOSE(MX X)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_3y
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L365
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L365-L367
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(59, varargin{:});
    end
    function varargout = vec(varargin)
    %VEC make a vector
    %
    %  Sparsity = VEC(Sparsity a)
    %  DM = VEC(DM a)
    %  SX = VEC(SX a)
    %  MX = VEC(MX a)
    %
    %
    %Reshapes/vectorizes the matrix such that the shape becomes 
    %(expr.numel(), 
    %1). Columns are stacked on top of each other. Same as 
    %reshape(expr, 
    %expr.numel(), 1)
    %
    %a c 
    %b d 
    % turns into
    %
    %a 
    %b 
    %c 
    %d 
    % Extra doc: https://github.com/casadi/casadi/wiki/L_3z
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L386
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L386-L388
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(60, varargin{:});
    end
    function varargout = reshape(varargin)
    %RESHAPE Reshape the matrix.
    %
    %  Sparsity = RESHAPE(Sparsity a, [int,int] rc)
    %  Sparsity = RESHAPE(Sparsity a, Sparsity sp)
    %  DM = RESHAPE(DM a, [int,int] rc)
    %  DM = RESHAPE(DM a, Sparsity sp)
    %  SX = RESHAPE(SX a, [int,int] rc)
    %  SX = RESHAPE(SX a, Sparsity sp)
    %  MX = RESHAPE(MX a, [int,int] rc)
    %  MX = RESHAPE(MX a, Sparsity sp)
    %  Sparsity = RESHAPE(Sparsity a, int nrow, int ncol)
    %  DM = RESHAPE(DM a, int nrow, int ncol)
    %  SX = RESHAPE(SX a, int nrow, int ncol)
    %  MX = RESHAPE(MX a, int nrow, int ncol)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_42
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L407
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L407-L409
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(61, varargin{:});
    end
    function varargout = sparsity_cast(varargin)
    %SPARSITY_CAST Cast matrix nonzeros to different Sparsity.
    %
    %  Sparsity = SPARSITY_CAST(Sparsity a, Sparsity sp)
    %  DM = SPARSITY_CAST(DM a, Sparsity sp)
    %  SX = SPARSITY_CAST(SX a, Sparsity sp)
    %  MX = SPARSITY_CAST(MX a, Sparsity sp)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_24z
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L414
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L414-L416
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(62, varargin{:});
    end
    function varargout = sprank(varargin)
    %SPRANK 
    %
    %  int = SPRANK(Sparsity A)
    %  int = SPRANK(DM A)
    %  int = SPRANK(SX A)
    %  int = SPRANK(MX A)
    %
    %
     [varargout{1:nargout}] = casadiMEX(63, varargin{:});
    end
    function varargout = norm_0_mul(varargin)
    %NORM_0_MUL 
    %
    %  int = NORM_0_MUL(Sparsity x, Sparsity y)
    %  int = NORM_0_MUL(DM x, DM y)
    %  int = NORM_0_MUL(SX x, SX y)
    %  int = NORM_0_MUL(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(64, varargin{:});
    end
    function varargout = triu(varargin)
    %TRIU Get the upper triangular part of a matrix.
    %
    %  Sparsity = TRIU(Sparsity a, bool includeDiagonal)
    %  DM = TRIU(DM a, bool includeDiagonal)
    %  SX = TRIU(SX a, bool includeDiagonal)
    %  MX = TRIU(MX a, bool includeDiagonal)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_45
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L435
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L435-L437
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(65, varargin{:});
    end
    function varargout = tril(varargin)
    %TRIL Get the lower triangular part of a matrix.
    %
    %  Sparsity = TRIL(Sparsity a, bool includeDiagonal)
    %  DM = TRIL(DM a, bool includeDiagonal)
    %  SX = TRIL(SX a, bool includeDiagonal)
    %  MX = TRIL(MX a, bool includeDiagonal)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_46
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L442
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L442-L444
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(66, varargin{:});
    end
    function varargout = kron(varargin)
    %KRON Kronecker tensor product.
    %
    %  Sparsity = KRON(Sparsity a, Sparsity b)
    %  DM = KRON(DM a, DM b)
    %  SX = KRON(SX a, SX b)
    %  MX = KRON(MX a, MX b)
    %
    %
    %Creates a block matrix in which each element (i, j) is a_ij*b
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_47
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L451
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L451-L453
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(67, varargin{:});
    end
    function varargout = repmat(varargin)
    %REPMAT Repeat matrix A n times vertically and m times horizontally.
    %
    %  Sparsity = REPMAT(Sparsity A, int n, int m)
    %  Sparsity = REPMAT(Sparsity A, [int,int] rc)
    %  DM = REPMAT(DM A, int n, int m)
    %  DM = REPMAT(DM A, [int,int] rc)
    %  SX = REPMAT(SX A, int n, int m)
    %  SX = REPMAT(SX A, [int,int] rc)
    %  MX = REPMAT(MX A, int n, int m)
    %  MX = REPMAT(MX A, [int,int] rc)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_49
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L465
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L465-L467
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(68, varargin{:});
    end
    function varargout = sum2(varargin)
    %SUM2 Return a column-wise summation of elements.
    %
    %  Sparsity = SUM2(Sparsity x)
    %  DM = SUM2(DM x)
    %  SX = SUM2(SX x)
    %  MX = SUM2(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_4q
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L591
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L591-L591
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(69, varargin{:});
    end
    function varargout = sum1(varargin)
    %SUM1 Return a row-wise summation of elements.
    %
    %  Sparsity = SUM1(Sparsity x)
    %  DM = SUM1(DM x)
    %  SX = SUM1(SX x)
    %  MX = SUM1(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_4p
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L586
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/sparsity_interface.hpp#L586-L586
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(70, varargin{:});
    end
    function varargout = length(varargin)
    %LENGTH 
    %
    %  int = LENGTH(Sparsity v)
    %  int = LENGTH(DM v)
    %  int = LENGTH(SX v)
    %  int = LENGTH(MX v)
    %
    %
     [varargout{1:nargout}] = casadiMEX(71, varargin{:});
    end
    function self = SparsityInterfaceCommon(varargin)
    %SPARSITYINTERFACECOMMON 
    %
    %  new_obj = SPARSITYINTERFACECOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(72, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(73, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
