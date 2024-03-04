classdef  Linsol < casadi.SharedObject & casadi.PrintableCommon
    %LINSOL [INTERNAL] 
    %
    %
    %Linear solver.
    %
    %Create a solver for linear systems of equations Solves the linear 
    %system 
    %A*X = B or A^T*X = B for X with A square and non-singular
    %
    %If A is structurally singular, an error will be thrown during init. If
    % A is
    % numerically singular, the prepare step will fail.
    %General informationList 
    %of plugins
    %- csparsecholesky
    %
    %- csparse
    %
    %- ma27
    %
    %- lapacklu
    %
    %- lapackqr
    %
    %- mumps
    %
    %- ldl
    %
    %- qr
    %
    %- tridiag
    %
    %- symbolicqr
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with   
    %Linsol.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %csparsecholesky
    %---------------
    %
    %
    %
    %Linsol with CSparseCholesky Interface
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21u
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %csparse
    %-------
    %
    %
    %
    %Linsol with CSparse Interface
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21t
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ma27
    %----
    %
    %
    %
    %Interface to the sparse direct linear solver MA27 Works for symmetric
    % 
    %indefinite systems Partly adopted from qpOASES 3.2 
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_229
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %lapacklu
    %--------
    %
    %
    %
    %This class solves the linear system  A.x=b by making an LU factorization of 
    %A:  A = L.U, with L lower and U upper triangular
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22h
    %
    %>List of available options
    %
    %+-----------------------------+---------+----------------------------------+
    %|             Id              |  Type   |           Description            |
    %+=============================+=========+==================================+
    %| allow_equilibration_failure | OT_BOOL | Non-fatal error when             |
    %|                             |         | equilibration fails              |
    %+-----------------------------+---------+----------------------------------+
    %| equilibration               | OT_BOOL | Equilibrate the matrix           |
    %+-----------------------------+---------+----------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %lapackqr
    %--------
    %
    %
    %
    %This class solves the linear system  A.x=b by making an QR factorization of 
    %A:  A = Q.R, with Q orthogonal and R upper triangular
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22g
    %
    %>List of available options
    %
    %+----------+--------+------------------------------------------------------+
    %|    Id    |  Type  |                     Description                      |
    %+==========+========+======================================================+
    %| max_nrhs | OT_INT | Maximum number of right-hand-sides that get          |
    %|          |        | processed in a single pass [default:10].             |
    %+----------+--------+------------------------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %mumps
    %-----
    %
    %
    %
    %Interface to the sparse direct linear solver MUMPS Works for 
    %symmetric 
    %indefinite systems 
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22t
    %
    %>List of available options
    %
    %+-----------+---------+-------------------+
    %|    Id     |  Type   |    Description    |
    %+===========+=========+===================+
    %| posdef    | OT_BOOL | Positive definite |
    %+-----------+---------+-------------------+
    %| symmetric | OT_BOOL | Symmetric matrix  |
    %+-----------+---------+-------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %ldl
    %---
    %
    %
    %
    %Linear solver using sparse direct LDL factorization
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_233
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %qr
    %--
    %
    %
    %
    %Linear solver using sparse direct QR factorization
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22z
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %tridiag
    %-------
    %
    %
    %
    %Linear solver for tridiagonal matrices
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22v
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %symbolicqr
    %----------
    %
    %
    %
    %Linear solver for sparse least-squares problems Inspired from 
    %https://github.com/scipy/scipy/blob/v0.14.0/scipy/sparse/linalg/isolve/lsqr.py#L96
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_230
    %
    %Linsol based on QR factorization with sparsity pattern based reordering  
    %without partial pivoting
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_231
    %
    %>List of available options
    %
    %+-------+---------+----------------------------------------------------+
    %|  Id   |  Type   |                    Description                     |
    %+=======+=========+====================================================+
    %| fopts | OT_DICT | Options to be passed to generated function objects |
    %+-------+---------+----------------------------------------------------+
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1kh
    %
    %C++ includes: linsol.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = plugin_name(self,varargin)
    %PLUGIN_NAME [INTERNAL] 
    %
    %  char = PLUGIN_NAME(self)
    %
    %Query plugin name.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L97
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L64-L66
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(856, self, varargin{:});
    end
    function varargout = sparsity(self,varargin)
    %SPARSITY [INTERNAL] 
    %
    %  Sparsity = SPARSITY(self)
    %
    %Get linear system sparsity.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L100
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L68-L70
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(857, self, varargin{:});
    end
    function varargout = sfact(self,varargin)
    %SFACT [INTERNAL] 
    %
    %  SFACT(self, DM A)
    %
    %Symbolic factorization of the linear system, e.g. selecting 
    %pivots.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L103
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L104-L107
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(858, self, varargin{:});
    end
    function varargout = nfact(self,varargin)
    %NFACT [INTERNAL] 
    %
    %  NFACT(self, DM A)
    %
    %Numeric factorization of the linear system.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L106
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L126-L129
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(859, self, varargin{:});
    end
    function varargout = solve(self,varargin)
    %SOLVE [INTERNAL] 
    %
    %  DM = SOLVE(self, DM A, DM B, bool tr)
    %  MX = SOLVE(self, MX A, MX B, bool tr)
    %
    % Solve linear system of equations
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L111
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L100-L102
    %
    %
    %
    %.......
    %
    %::
    %
    %  SOLVE(self, DM A, DM B, bool tr)
    %
    %
    %
    %[INTERNAL] 
    % Solve linear system of equations
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L110
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L72-L98
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
    %  SOLVE(self, MX A, MX B, bool tr)
    %
    %
    %
    %[INTERNAL] 
    % Solve linear system of equations
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L111
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L100-L102
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(860, self, varargin{:});
    end
    function varargout = neig(self,varargin)
    %NEIG [INTERNAL] 
    %
    %  int = NEIG(self, DM A)
    %
    %Number of negative eigenvalues.
    %
    %Not available for all solvers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1kk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L119
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L165-L170
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(861, self, varargin{:});
    end
    function varargout = rank(self,varargin)
    %RANK [INTERNAL] 
    %
    %  int = RANK(self, DM A)
    %
    % Matrix rank.
    %
    %Not available for all solvers
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1kl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L126
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L176-L181
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(862, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS [INTERNAL] 
    %
    %  struct = STATS(self, int mem)
    %
    %Get all statistics obtained at the end of the last evaluate 
    %call.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L129
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L217-L219
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(863, self, varargin{:});
    end
    function self = Linsol(varargin)
    %LINSOL 
    %
    %  new_obj = LINSOL()
    %  new_obj = LINSOL(char name, char solver, Sparsity sp, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  LINSOL()
    %
    %
    %
    %[INTERNAL] 
    %Default constructor.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L63
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L31-L32
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
    %  LINSOL(char name, char solver, Sparsity sp, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Constructor.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.hpp#L66
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/linsol.cpp#L34-L38
    %
    %
    %
    %.............
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(864, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(865, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(852, varargin{:});
    end
    function varargout = has_plugin(varargin)
    %HAS_PLUGIN 
    %
    %  bool = HAS_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(853, varargin{:});
    end
    function varargout = load_plugin(varargin)
    %LOAD_PLUGIN 
    %
    %  LOAD_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(854, varargin{:});
    end
    function varargout = doc(varargin)
    %DOC 
    %
    %  char = DOC(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(855, varargin{:});
    end
  end
end
