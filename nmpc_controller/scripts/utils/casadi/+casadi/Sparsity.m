classdef  Sparsity < casadi.SharedObject & casadi.SparsityInterfaceCommon & casadi.PrintableCommon
    %SPARSITY 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = permutation_vector(self,varargin)
    %PERMUTATION_VECTOR 
    %
    %  [int] = PERMUTATION_VECTOR(self, bool invert)
    %
    %
      [varargout{1:nargout}] = casadiMEX(87, self, varargin{:});
    end
    function varargout = get_diag(self,varargin)
    %GET_DIAG 
    %
    %  [Sparsity , [int] OUTPUT] = GET_DIAG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(88, self, varargin{:});
    end
    function varargout = compress(self,varargin)
    %COMPRESS 
    %
    %  [int] = COMPRESS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(89, self, varargin{:});
    end
    function varargout = is_equal(self,varargin)
    %IS_EQUAL 
    %
    %  bool = IS_EQUAL(self, Sparsity y)
    %  bool = IS_EQUAL(self, int nrow, int ncol, [int] colind, [int] row)
    %
    %
      [varargout{1:nargout}] = casadiMEX(90, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(91, self, varargin{:});
    end
    function varargout = ne(self,varargin)
    %NE 
    %
    %  bool = NE(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(92, self, varargin{:});
    end
    function varargout = is_stacked(self,varargin)
    %IS_STACKED 
    %
    %  bool = IS_STACKED(self, Sparsity y, int n)
    %
    %
      [varargout{1:nargout}] = casadiMEX(93, self, varargin{:});
    end
    function varargout = size1(self,varargin)
    %SIZE1 
    %
    %  int = SIZE1(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(94, self, varargin{:});
    end
    function varargout = rows(self,varargin)
    %ROWS 
    %
    %  int = ROWS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(95, self, varargin{:});
    end
    function varargout = size2(self,varargin)
    %SIZE2 
    %
    %  int = SIZE2(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(96, self, varargin{:});
    end
    function varargout = columns(self,varargin)
    %COLUMNS 
    %
    %  int = COLUMNS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(97, self, varargin{:});
    end
    function varargout = numel(self,varargin)
    %NUMEL 
    %
    %  int = NUMEL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(98, self, varargin{:});
    end
    function varargout = density(self,varargin)
    %DENSITY 
    %
    %  double = DENSITY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(99, self, varargin{:});
    end
    function varargout = is_empty(self,varargin)
    %IS_EMPTY 
    %
    %  bool = IS_EMPTY(self, bool both)
    %
    %
      [varargout{1:nargout}] = casadiMEX(100, self, varargin{:});
    end
    function varargout = nnz(self,varargin)
    %NNZ 
    %
    %  int = NNZ(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(101, self, varargin{:});
    end
    function varargout = nnz_upper(self,varargin)
    %NNZ_UPPER 
    %
    %  int = NNZ_UPPER(self, bool strictly)
    %
    %
      [varargout{1:nargout}] = casadiMEX(102, self, varargin{:});
    end
    function varargout = nnz_lower(self,varargin)
    %NNZ_LOWER 
    %
    %  int = NNZ_LOWER(self, bool strictly)
    %
    %
      [varargout{1:nargout}] = casadiMEX(103, self, varargin{:});
    end
    function varargout = nnz_diag(self,varargin)
    %NNZ_DIAG 
    %
    %  int = NNZ_DIAG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(104, self, varargin{:});
    end
    function varargout = bw_upper(self,varargin)
    %BW_UPPER 
    %
    %  int = BW_UPPER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(105, self, varargin{:});
    end
    function varargout = bw_lower(self,varargin)
    %BW_LOWER 
    %
    %  int = BW_LOWER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(106, self, varargin{:});
    end
    function varargout = size(self,varargin)
    %SIZE 
    %
    %  [int,int] = SIZE(self)
    %  int = SIZE(self, int axis)
    %
    %
      out = casadiMEX(107, self, varargin{:});
      if nargout<=1
        varargout{1}=out;
      else
        nargoutchk(length(out),length(out))
        for i=1:nargout
          varargout{i} = out(i);
        end
      end
    end
    function varargout = info(self,varargin)
    %INFO 
    %
    %  struct = INFO(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(108, self, varargin{:});
    end
    function varargout = to_file(self,varargin)
    %TO_FILE 
    %
    %  TO_FILE(self, char filename, char format_hint)
    %
    %
      [varargout{1:nargout}] = casadiMEX(109, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE 
    %
    %  char = SERIALIZE(self)
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %
      [varargout{1:nargout}] = casadiMEX(111, self, varargin{:});
    end
    function varargout = colind(self,varargin)
    %COLIND 
    %
    %  [int] = COLIND(self)
    %  int = COLIND(self, int cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(113, self, varargin{:});
    end
    function varargout = row(self,varargin)
    %ROW 
    %
    %  [int] = ROW(self)
    %  int = ROW(self, int el)
    %
    %
      [varargout{1:nargout}] = casadiMEX(114, self, varargin{:});
    end
    function varargout = get_col(self,varargin)
    %GET_COL 
    %
    %  [int] = GET_COL(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(115, self, varargin{:});
    end
    function varargout = resize(self,varargin)
    %RESIZE 
    %
    %  RESIZE(self, int nrow, int ncol)
    %
    %
      [varargout{1:nargout}] = casadiMEX(116, self, varargin{:});
    end
    function varargout = add_nz(self,varargin)
    %ADD_NZ 
    %
    %  int = ADD_NZ(self, int rr, int cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(117, self, varargin{:});
    end
    function varargout = has_nz(self,varargin)
    %HAS_NZ 
    %
    %  bool = HAS_NZ(self, int rr, int cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(118, self, varargin{:});
    end
    function varargout = get_nz(self,varargin)
    %GET_NZ 
    %
    %  [int] = GET_NZ(self)
    %  int = GET_NZ(self, int rr, int cc)
    %  [int] = GET_NZ(self, [int] rr, [int] cc)
    %
    %
      [varargout{1:nargout}] = casadiMEX(119, self, varargin{:});
    end
    function varargout = get_lower(self,varargin)
    %GET_LOWER 
    %
    %  [int] = GET_LOWER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(120, self, varargin{:});
    end
    function varargout = get_upper(self,varargin)
    %GET_UPPER 
    %
    %  [int] = GET_UPPER(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(121, self, varargin{:});
    end
    function varargout = get_ccs(self,varargin)
    %GET_CCS 
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_CCS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(122, self, varargin{:});
    end
    function varargout = get_crs(self,varargin)
    %GET_CRS 
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_CRS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(123, self, varargin{:});
    end
    function varargout = get_triplet(self,varargin)
    %GET_TRIPLET 
    %
    %  [[int] OUTPUT, [int] OUTPUT] = GET_TRIPLET(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(124, self, varargin{:});
    end
    function varargout = sub(self,varargin)
    %SUB 
    %
    %  [Sparsity , [int] OUTPUT] = SUB(self, [int] rr, Sparsity sp, bool ind1)
    %  [Sparsity , [int] OUTPUT] = SUB(self, [int] rr, [int] cc, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(125, self, varargin{:});
    end
    function varargout = T(self,varargin)
    %T 
    %
    %  Sparsity = T(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(126, self, varargin{:});
    end
    function varargout = transpose(self,varargin)
    %TRANSPOSE 
    %
    %  [Sparsity , [int] OUTPUT] = TRANSPOSE(self, bool invert_mapping)
    %
    %
      [varargout{1:nargout}] = casadiMEX(127, self, varargin{:});
    end
    function varargout = is_transpose(self,varargin)
    %IS_TRANSPOSE 
    %
    %  bool = IS_TRANSPOSE(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(128, self, varargin{:});
    end
    function varargout = is_reshape(self,varargin)
    %IS_RESHAPE 
    %
    %  bool = IS_RESHAPE(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(129, self, varargin{:});
    end
    function varargout = combine(self,varargin)
    %COMBINE 
    %
    %  Sparsity = COMBINE(self, Sparsity y, bool f0x_is_zero, bool function0_is_zero)
    %
    %
      [varargout{1:nargout}] = casadiMEX(130, self, varargin{:});
    end
    function varargout = unite(self,varargin)
    %UNITE 
    %
    %  Sparsity = UNITE(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(131, self, varargin{:});
    end
    function varargout = plus(self,varargin)
    %PLUS 
    %
    %  Sparsity = PLUS(self, Sparsity b)
    %
    %
      [varargout{1:nargout}] = casadiMEX(132, self, varargin{:});
    end
    function varargout = intersect(self,varargin)
    %INTERSECT 
    %
    %  Sparsity = INTERSECT(self, Sparsity y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(133, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
    %MTIMES 
    %
    %  Sparsity = MTIMES(self, Sparsity b)
    %
    %
      [varargout{1:nargout}] = casadiMEX(134, self, varargin{:});
    end
    function varargout = is_subset(self,varargin)
    %IS_SUBSET 
    %
    %  bool = IS_SUBSET(self, Sparsity rhs)
    %
    %
      [varargout{1:nargout}] = casadiMEX(135, self, varargin{:});
    end
    function varargout = sparsity_cast_mod(self,varargin)
    %SPARSITY_CAST_MOD 
    %
    %  Sparsity = SPARSITY_CAST_MOD(self, Sparsity X, Sparsity Y)
    %
    %
      [varargout{1:nargout}] = casadiMEX(136, self, varargin{:});
    end
    function varargout = pattern_inverse(self,varargin)
    %PATTERN_INVERSE 
    %
    %  Sparsity = PATTERN_INVERSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(137, self, varargin{:});
    end
    function varargout = enlarge(self,varargin)
    %ENLARGE 
    %
    %  ENLARGE(self, int nrow, int ncol, [int] rr, [int] cc, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(138, self, varargin{:});
    end
    function varargout = enlargeRows(self,varargin)
    %ENLARGEROWS 
    %
    %  ENLARGEROWS(self, int nrow, [int] rr, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(139, self, varargin{:});
    end
    function varargout = enlargeColumns(self,varargin)
    %ENLARGECOLUMNS 
    %
    %  ENLARGECOLUMNS(self, int ncol, [int] cc, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(140, self, varargin{:});
    end
    function varargout = makeDense(self,varargin)
    %MAKEDENSE 
    %
    %  [Sparsity , [int] OUTPUT] = MAKEDENSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(141, self, varargin{:});
    end
    function varargout = erase(self,varargin)
    %ERASE 
    %
    %  [int] = ERASE(self, [int] rr, bool ind1)
    %  [int] = ERASE(self, [int] rr, [int] cc, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(142, self, varargin{:});
    end
    function varargout = append(self,varargin)
    %APPEND 
    %
    %  APPEND(self, Sparsity sp)
    %
    %
      [varargout{1:nargout}] = casadiMEX(143, self, varargin{:});
    end
    function varargout = appendColumns(self,varargin)
    %APPENDCOLUMNS 
    %
    %  APPENDCOLUMNS(self, Sparsity sp)
    %
    %
      [varargout{1:nargout}] = casadiMEX(144, self, varargin{:});
    end
    function varargout = is_scalar(self,varargin)
    %IS_SCALAR 
    %
    %  bool = IS_SCALAR(self, bool scalar_and_dense)
    %
    %
      [varargout{1:nargout}] = casadiMEX(145, self, varargin{:});
    end
    function varargout = is_dense(self,varargin)
    %IS_DENSE 
    %
    %  bool = IS_DENSE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(146, self, varargin{:});
    end
    function varargout = is_row(self,varargin)
    %IS_ROW 
    %
    %  bool = IS_ROW(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(147, self, varargin{:});
    end
    function varargout = is_column(self,varargin)
    %IS_COLUMN 
    %
    %  bool = IS_COLUMN(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(148, self, varargin{:});
    end
    function varargout = is_vector(self,varargin)
    %IS_VECTOR 
    %
    %  bool = IS_VECTOR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(149, self, varargin{:});
    end
    function varargout = is_diag(self,varargin)
    %IS_DIAG 
    %
    %  bool = IS_DIAG(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(150, self, varargin{:});
    end
    function varargout = is_square(self,varargin)
    %IS_SQUARE 
    %
    %  bool = IS_SQUARE(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(151, self, varargin{:});
    end
    function varargout = is_symmetric(self,varargin)
    %IS_SYMMETRIC 
    %
    %  bool = IS_SYMMETRIC(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(152, self, varargin{:});
    end
    function varargout = is_triu(self,varargin)
    %IS_TRIU 
    %
    %  bool = IS_TRIU(self, bool strictly)
    %
    %
      [varargout{1:nargout}] = casadiMEX(153, self, varargin{:});
    end
    function varargout = is_tril(self,varargin)
    %IS_TRIL 
    %
    %  bool = IS_TRIL(self, bool strictly)
    %
    %
      [varargout{1:nargout}] = casadiMEX(154, self, varargin{:});
    end
    function varargout = is_singular(self,varargin)
    %IS_SINGULAR 
    %
    %  bool = IS_SINGULAR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(155, self, varargin{:});
    end
    function varargout = is_permutation(self,varargin)
    %IS_PERMUTATION 
    %
    %  bool = IS_PERMUTATION(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(156, self, varargin{:});
    end
    function varargout = is_selection(self,varargin)
    %IS_SELECTION 
    %
    %  bool = IS_SELECTION(self, bool allow_empty)
    %
    %
      [varargout{1:nargout}] = casadiMEX(157, self, varargin{:});
    end
    function varargout = is_orthonormal(self,varargin)
    %IS_ORTHONORMAL 
    %
    %  bool = IS_ORTHONORMAL(self, bool allow_empty)
    %
    %
      [varargout{1:nargout}] = casadiMEX(158, self, varargin{:});
    end
    function varargout = is_orthonormal_rows(self,varargin)
    %IS_ORTHONORMAL_ROWS 
    %
    %  bool = IS_ORTHONORMAL_ROWS(self, bool allow_empty)
    %
    %
      [varargout{1:nargout}] = casadiMEX(159, self, varargin{:});
    end
    function varargout = is_orthonormal_columns(self,varargin)
    %IS_ORTHONORMAL_COLUMNS 
    %
    %  bool = IS_ORTHONORMAL_COLUMNS(self, bool allow_empty)
    %
    %
      [varargout{1:nargout}] = casadiMEX(160, self, varargin{:});
    end
    function varargout = rowsSequential(self,varargin)
    %ROWSSEQUENTIAL 
    %
    %  bool = ROWSSEQUENTIAL(self, bool strictly)
    %
    %
      [varargout{1:nargout}] = casadiMEX(161, self, varargin{:});
    end
    function varargout = removeDuplicates(self,varargin)
    %REMOVEDUPLICATES 
    %
    %  [int] = REMOVEDUPLICATES(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(162, self, varargin{:});
    end
    function varargout = etree(self,varargin)
    %ETREE 
    %
    %  [int] = ETREE(self, bool ata)
    %
    %
      [varargout{1:nargout}] = casadiMEX(163, self, varargin{:});
    end
    function varargout = ldl(self,varargin)
    %LDL 
    %
    %  [Sparsity , [int] OUTPUT] = LDL(self, bool amd)
    %
    %
      [varargout{1:nargout}] = casadiMEX(164, self, varargin{:});
    end
    function varargout = qr_sparse(self,varargin)
    %QR_SPARSE 
    %
    %  [Sparsity OUTPUT, Sparsity OUTPUT, [int] OUTPUT, [int] OUTPUT] = QR_SPARSE(self, bool amd)
    %
    %
      [varargout{1:nargout}] = casadiMEX(165, self, varargin{:});
    end
    function varargout = dfs(self,varargin)
    %DFS 
    %
    %  [int , [int] INOUT, [int] INOUT, [bool] INOUT] = DFS(self, int j, int top, [int] pinv)
    %
    %
      [varargout{1:nargout}] = casadiMEX(166, self, varargin{:});
    end
    function varargout = scc(self,varargin)
    %SCC 
    %
    %  [int , [int] OUTPUT, [int] OUTPUT] = SCC(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(167, self, varargin{:});
    end
    function varargout = btf(self,varargin)
    %BTF 
    %
    %  [int , [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT, [int] OUTPUT] = BTF(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(168, self, varargin{:});
    end
    function varargout = amd(self,varargin)
    %AMD 
    %
    %  [int] = AMD(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(169, self, varargin{:});
    end
    function varargout = find(self,varargin)
    %FIND 
    %
    %  [int] = FIND(self, bool ind1)
    %
    %
      [varargout{1:nargout}] = casadiMEX(170, self, varargin{:});
    end
    function varargout = uni_coloring(self,varargin)
    %UNI_COLORING 
    %
    %  Sparsity = UNI_COLORING(self, Sparsity AT, int cutoff)
    %
    %
      [varargout{1:nargout}] = casadiMEX(171, self, varargin{:});
    end
    function varargout = star_coloring(self,varargin)
    %STAR_COLORING 
    %
    %  Sparsity = STAR_COLORING(self, int ordering, int cutoff)
    %
    %
      [varargout{1:nargout}] = casadiMEX(172, self, varargin{:});
    end
    function varargout = star_coloring2(self,varargin)
    %STAR_COLORING2 
    %
    %  Sparsity = STAR_COLORING2(self, int ordering, int cutoff)
    %
    %
      [varargout{1:nargout}] = casadiMEX(173, self, varargin{:});
    end
    function varargout = largest_first(self,varargin)
    %LARGEST_FIRST 
    %
    %  [int] = LARGEST_FIRST(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(174, self, varargin{:});
    end
    function varargout = pmult(self,varargin)
    %PMULT 
    %
    %  Sparsity = PMULT(self, [int] p, bool permute_rows, bool permute_columns, bool invert_permutation)
    %
    %
      [varargout{1:nargout}] = casadiMEX(175, self, varargin{:});
    end
    function varargout = dim(self,varargin)
    %DIM 
    %
    %  char = DIM(self, bool with_nz)
    %
    %
      [varargout{1:nargout}] = casadiMEX(176, self, varargin{:});
    end
    function varargout = postfix_dim(self,varargin)
    %POSTFIX_DIM 
    %
    %  char = POSTFIX_DIM(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(177, self, varargin{:});
    end
    function varargout = repr_el(self,varargin)
    %REPR_EL 
    %
    %  char = REPR_EL(self, int k)
    %
    %
      [varargout{1:nargout}] = casadiMEX(178, self, varargin{:});
    end
    function varargout = spy(self,varargin)
    %SPY 
    %
    %  std::ostream & = SPY(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(179, self, varargin{:});
    end
    function varargout = spy_matlab(self,varargin)
    %SPY_MATLAB 
    %
    %  SPY_MATLAB(self, char mfile)
    %
    %
      [varargout{1:nargout}] = casadiMEX(180, self, varargin{:});
    end
    function varargout = export_code(self,varargin)
    %EXPORT_CODE 
    %
    %  std::ostream & = EXPORT_CODE(self, char lang, struct options)
    %
    %
      [varargout{1:nargout}] = casadiMEX(181, self, varargin{:});
    end
    function varargout = hash(self,varargin)
    %HASH 
    %
    %  std::size_t = HASH(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(183, self, varargin{:});
    end

     function s = saveobj(obj)
        try
            s.serialization = obj.serialize();
        catch exception
            warning(['Serializing of CasADi Sparsity failed:' getReport(exception) ]);
            s = struct;
        end
     end
      function self = Sparsity(varargin)
    %SPARSITY 
    %
    %  new_obj = SPARSITY(int dummy)
    %  new_obj = SPARSITY([int,int] rc)
    %  new_obj = SPARSITY(int nrow, int ncol)
    %  new_obj = SPARSITY(int nrow, int ncol, [int] colind, [int] row, bool order_rows)
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.SparsityInterfaceCommon(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(186, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(187, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
    function varargout = scalar(varargin)
    %SCALAR 
    %
    %  Sparsity = SCALAR(bool dense_scalar)
    %
    %
     [varargout{1:nargout}] = casadiMEX(74, varargin{:});
    end
    function varargout = dense(varargin)
    %DENSE 
    %
    %  Sparsity = DENSE(int nrow, int ncol)
    %  Sparsity = DENSE([int,int] rc)
    %
    %
     [varargout{1:nargout}] = casadiMEX(75, varargin{:});
    end
    function varargout = unit(varargin)
    %UNIT 
    %
    %  Sparsity = UNIT(int n, int el)
    %
    %
     [varargout{1:nargout}] = casadiMEX(76, varargin{:});
    end
    function varargout = upper(varargin)
    %UPPER 
    %
    %  Sparsity = UPPER(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(77, varargin{:});
    end
    function varargout = lower(varargin)
    %LOWER 
    %
    %  Sparsity = LOWER(int n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(78, varargin{:});
    end
    function varargout = diag(varargin)
    %DIAG 
    %
    %  Sparsity = DIAG(int nrow)
    %  Sparsity = DIAG([int,int] rc)
    %  Sparsity = DIAG(int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(79, varargin{:});
    end
    function varargout = band(varargin)
    %BAND 
    %
    %  Sparsity = BAND(int n, int p)
    %
    %
     [varargout{1:nargout}] = casadiMEX(80, varargin{:});
    end
    function varargout = banded(varargin)
    %BANDED 
    %
    %  Sparsity = BANDED(int n, int p)
    %
    %
     [varargout{1:nargout}] = casadiMEX(81, varargin{:});
    end
    function varargout = rowcol(varargin)
    %ROWCOL 
    %
    %  Sparsity = ROWCOL([int] row, [int] col, int nrow, int ncol)
    %
    %
     [varargout{1:nargout}] = casadiMEX(82, varargin{:});
    end
    function varargout = triplet(varargin)
    %TRIPLET 
    %
    %  Sparsity = TRIPLET(int nrow, int ncol, [int] row, [int] col)
    %  [Sparsity , [int] OUTPUT] = TRIPLET(int nrow, int ncol, [int] row, [int] col, bool invert_mapping)
    %
    %
     [varargout{1:nargout}] = casadiMEX(83, varargin{:});
    end
    function varargout = nonzeros(varargin)
    %NONZEROS 
    %
    %  Sparsity = NONZEROS(int nrow, int ncol, [int] nz, bool ind1)
    %
    %
     [varargout{1:nargout}] = casadiMEX(84, varargin{:});
    end
    function varargout = compressed(varargin)
    %COMPRESSED 
    %
    %  Sparsity = COMPRESSED([int] v, bool order_rows)
    %
    %
     [varargout{1:nargout}] = casadiMEX(85, varargin{:});
    end
    function varargout = permutation(varargin)
    %PERMUTATION 
    %
    %  Sparsity = PERMUTATION([int] p, bool invert)
    %
    %
     [varargout{1:nargout}] = casadiMEX(86, varargin{:});
    end
    function varargout = from_file(varargin)
    %FROM_FILE 
    %
    %  Sparsity = FROM_FILE(char filename, char format_hint)
    %
    %
     [varargout{1:nargout}] = casadiMEX(110, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Sparsity = DESERIALIZE(std::istream & stream)
    %  Sparsity = DESERIALIZE(casadi::DeserializingStream & s)
    %  Sparsity = DESERIALIZE(char s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(112, varargin{:});
    end
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(182, varargin{:});
    end
    function varargout = test_cast(varargin)
    %TEST_CAST 
    %
    %  bool = TEST_CAST(casadi::SharedObjectInternal const * ptr)
    %
    %
     [varargout{1:nargout}] = casadiMEX(184, varargin{:});
    end
    function varargout = kkt(varargin)
    %KKT 
    %
    %  Sparsity = KKT(Sparsity H, Sparsity J, bool with_x_diag, bool with_lam_g_diag)
    %
    %
     [varargout{1:nargout}] = casadiMEX(185, varargin{:});
    end

     function obj = loadobj(s)
        try
          if isstruct(s)
             obj = casadi.Sparsity.deserialize(s.serialization);
          else
             obj = s;
          end
        catch exception
            warning(['Serializing of CasADi Sparsity failed:' getReport(exception) ]);
            s = struct;
        end
     end
    end
end
