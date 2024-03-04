classdef  GenericExpressionCommon < SwigRef
    %GENERICEXPRESSIONCOMMON [INTERNAL] 
    %
    %   = GENERICEXPRESSIONCOMMON()
    %
    %Expression interface.
    %
    %This is a common base class for SX,  MX and Matrix<>, introducing a uniform 
    %syntax and implementing common 
    %functionality using the curiously recurring 
    %template pattern (CRTP) 
    %idiom.
    %Joel Andersson
    %
    %C++ includes: generic_expression.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = plus(varargin)
    %PLUS 
    %
    %  double = PLUS(double x, double y)
    %  DM = PLUS(DM x, DM y)
    %  SX = PLUS(SX x, SX y)
    %  MX = PLUS(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(362, varargin{:});
    end
    function varargout = minus(varargin)
    %MINUS Subtraction: (x,y) -> x - y.
    %
    %  double = MINUS(double x, double y)
    %  DM = MINUS(DM x, DM y)
    %  SX = MINUS(SX x, SX y)
    %  MX = MINUS(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oo
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L83
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L83-L85
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(363, varargin{:});
    end
    function varargout = times(varargin)
    %TIMES Elementwise multiplication: (x,y) -> x .* y.
    %
    %  double = TIMES(double x, double y)
    %  DM = TIMES(DM x, DM y)
    %  SX = TIMES(SX x, SX y)
    %  MX = TIMES(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_op
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L99
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L99-L101
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(364, varargin{:});
    end
    function varargout = rdivide(varargin)
    %RDIVIDE Elementwise division: (x,y) -> x ./ y.
    %
    %  double = RDIVIDE(double x, double y)
    %  DM = RDIVIDE(DM x, DM y)
    %  SX = RDIVIDE(SX x, SX y)
    %  MX = RDIVIDE(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_oq
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L115
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L115-L117
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(365, varargin{:});
    end
    function varargout = ldivide(varargin)
    %LDIVIDE 
    %
    %  double = LDIVIDE(double x, double y)
    %  DM = LDIVIDE(DM x, DM y)
    %  SX = LDIVIDE(SX x, SX y)
    %  MX = LDIVIDE(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(366, varargin{:});
    end
    function varargout = lt(varargin)
    %LT Logical less than: (x,y) -> x < y.
    %
    %  double = LT(double x, double y)
    %  DM = LT(DM x, DM y)
    %  SX = LT(SX x, SX y)
    %  MX = LT(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_or
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L131
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L131-L133
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(367, varargin{:});
    end
    function varargout = le(varargin)
    %LE Logical less or equal to: (x,y) -> x <= y.
    %
    %  double = LE(double x, double y)
    %  DM = LE(DM x, DM y)
    %  SX = LE(SX x, SX y)
    %  MX = LE(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_os
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L146
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L146-L148
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(368, varargin{:});
    end
    function varargout = gt(varargin)
    %GT Logical greater than: (x,y) -> x > y.
    %
    %  double = GT(double x, double y)
    %  DM = GT(DM x, DM y)
    %  SX = GT(SX x, SX y)
    %  MX = GT(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ot
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L161
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L161-L163
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(369, varargin{:});
    end
    function varargout = ge(varargin)
    %GE Logical greater or equal to: (x,y) -> x <= y.
    %
    %  double = GE(double x, double y)
    %  DM = GE(DM x, DM y)
    %  SX = GE(SX x, SX y)
    %  MX = GE(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ou
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L176
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L176-L178
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(370, varargin{:});
    end
    function varargout = eq(varargin)
    %EQ Logical equal to: (x,y) -> x == y.
    %
    %  double = EQ(double x, double y)
    %  DM = EQ(DM x, DM y)
    %  SX = EQ(SX x, SX y)
    %  MX = EQ(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ov
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L191
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L191-L193
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(371, varargin{:});
    end
    function varargout = ne(varargin)
    %NE Logical not equal to: (x,y) -> x != y.
    %
    %  double = NE(double x, double y)
    %  DM = NE(DM x, DM y)
    %  SX = NE(SX x, SX y)
    %  MX = NE(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ow
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L206
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L206-L208
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(372, varargin{:});
    end
    function varargout = and(varargin)
    %AND 
    %
    %  double = AND(double x, double y)
    %  DM = AND(DM x, DM y)
    %  SX = AND(SX x, SX y)
    %  MX = AND(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(373, varargin{:});
    end
    function varargout = or(varargin)
    %OR 
    %
    %  double = OR(double x, double y)
    %  DM = OR(DM x, DM y)
    %  SX = OR(SX x, SX y)
    %  MX = OR(MX x, MX y)
    %
    %
     [varargout{1:nargout}] = casadiMEX(374, varargin{:});
    end
    function varargout = not(varargin)
    %NOT 
    %
    %  double = NOT(double x)
    %  DM = NOT(DM x)
    %  SX = NOT(SX x)
    %  MX = NOT(MX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(375, varargin{:});
    end
    function varargout = abs(varargin)
    %ABS Absolute value: x -> abs(x)
    %
    %  double = ABS(double x)
    %  DM = ABS(DM x)
    %  SX = ABS(SX x)
    %  MX = ABS(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L275
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L275-L277
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(376, varargin{:});
    end
    function varargout = sqrt(varargin)
    %SQRT Square root: x -> sqrt(x)
    %
    %  double = SQRT(double x)
    %  DM = SQRT(DM x)
    %  SX = SQRT(SX x)
    %  MX = SQRT(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L290
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L290-L292
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(377, varargin{:});
    end
    function varargout = sin(varargin)
    %SIN Sine: x -> sin(x)
    %
    %  double = SIN(double x)
    %  DM = SIN(DM x)
    %  SX = SIN(SX x)
    %  MX = SIN(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p3
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L314
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L314-L316
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(378, varargin{:});
    end
    function varargout = cos(varargin)
    %COS Cosine: x -> cos(x)
    %
    %  double = COS(double x)
    %  DM = COS(DM x)
    %  SX = COS(SX x)
    %  MX = COS(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p4
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L326
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L326-L328
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(379, varargin{:});
    end
    function varargout = tan(varargin)
    %TAN Tangent: x -> tan(x)
    %
    %  double = TAN(double x)
    %  DM = TAN(DM x)
    %  SX = TAN(SX x)
    %  MX = TAN(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p5
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L338
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L338-L340
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(380, varargin{:});
    end
    function varargout = atan(varargin)
    %ATAN Arc tangent: x -> atan(x)
    %
    %  double = ATAN(double x)
    %  DM = ATAN(DM x)
    %  SX = ATAN(SX x)
    %  MX = ATAN(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p6
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L350
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L350-L352
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(381, varargin{:});
    end
    function varargout = asin(varargin)
    %ASIN Arc sine: x -> asin(x)
    %
    %  double = ASIN(double x)
    %  DM = ASIN(DM x)
    %  SX = ASIN(SX x)
    %  MX = ASIN(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p7
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L362
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L362-L364
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(382, varargin{:});
    end
    function varargout = acos(varargin)
    %ACOS Arc cosine: x -> acos(x)
    %
    %  double = ACOS(double x)
    %  DM = ACOS(DM x)
    %  SX = ACOS(SX x)
    %  MX = ACOS(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p8
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L374
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L374-L376
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(383, varargin{:});
    end
    function varargout = tanh(varargin)
    %TANH Hyperbolic tangent: x -> tanh(x)
    %
    %  double = TANH(double x)
    %  DM = TANH(DM x)
    %  SX = TANH(SX x)
    %  MX = TANH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_p9
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L386
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L386-L388
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(384, varargin{:});
    end
    function varargout = sinh(varargin)
    %SINH Hyperbolic sin: x -> sinh(x)
    %
    %  double = SINH(double x)
    %  DM = SINH(DM x)
    %  SX = SINH(SX x)
    %  MX = SINH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pa
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L398
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L398-L400
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(385, varargin{:});
    end
    function varargout = cosh(varargin)
    %COSH Hyperbolic cosine: x -> cosh(x)
    %
    %  double = COSH(double x)
    %  DM = COSH(DM x)
    %  SX = COSH(SX x)
    %  MX = COSH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pb
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L410
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L410-L412
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(386, varargin{:});
    end
    function varargout = atanh(varargin)
    %ATANH Inverse hyperbolic tangent: x -> atanh(x)
    %
    %  double = ATANH(double x)
    %  DM = ATANH(DM x)
    %  SX = ATANH(SX x)
    %  MX = ATANH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pc
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L422
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L422-L424
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(387, varargin{:});
    end
    function varargout = asinh(varargin)
    %ASINH Inverse hyperbolic sin: x -> asinh(x)
    %
    %  double = ASINH(double x)
    %  DM = ASINH(DM x)
    %  SX = ASINH(SX x)
    %  MX = ASINH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pd
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L434
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L434-L436
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(388, varargin{:});
    end
    function varargout = acosh(varargin)
    %ACOSH Inverse hyperbolic cosine: x -> acosh(x)
    %
    %  double = ACOSH(double x)
    %  DM = ACOSH(DM x)
    %  SX = ACOSH(SX x)
    %  MX = ACOSH(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pe
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L446
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L446-L448
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(389, varargin{:});
    end
    function varargout = exp(varargin)
    %EXP Elementwise exponential: x -> exp(x)
    %
    %  double = EXP(double x)
    %  DM = EXP(DM x)
    %  SX = EXP(SX x)
    %  MX = EXP(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pf
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L458
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L458-L460
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(390, varargin{:});
    end
    function varargout = log(varargin)
    %LOG Natural logarithm: x -> log(x)
    %
    %  double = LOG(double x)
    %  DM = LOG(DM x)
    %  SX = LOG(SX x)
    %  MX = LOG(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L470
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L470-L472
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(391, varargin{:});
    end
    function varargout = log10(varargin)
    %LOG10 Base-10 logarithm: x -> log10(x)
    %
    %  double = LOG10(double x)
    %  DM = LOG10(DM x)
    %  SX = LOG10(SX x)
    %  MX = LOG10(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ph
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L482
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L482-L484
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(392, varargin{:});
    end
    function varargout = log1p(varargin)
    %LOG1P Precision variant for natural logarithm: x -> log(x+1)
    %
    %  double = LOG1P(double x)
    %  DM = LOG1P(DM x)
    %  SX = LOG1P(SX x)
    %  MX = LOG1P(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pi
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L494
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L494-L496
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(393, varargin{:});
    end
    function varargout = expm1(varargin)
    %EXPM1 Precision variant for elementwise exponential: x -> exp(x)-1.
    %
    %  double = EXPM1(double x)
    %  DM = EXPM1(DM x)
    %  SX = EXPM1(SX x)
    %  MX = EXPM1(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pj
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L506
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L506-L508
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(394, varargin{:});
    end
    function varargout = floor(varargin)
    %FLOOR Round down to nearest integer: x -> floor(x)
    %
    %  double = FLOOR(double x)
    %  DM = FLOOR(DM x)
    %  SX = FLOOR(SX x)
    %  MX = FLOOR(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pk
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L518
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L518-L520
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(395, varargin{:});
    end
    function varargout = ceil(varargin)
    %CEIL Round up to nearest integer: x -> ceil(x)
    %
    %  double = CEIL(double x)
    %  DM = CEIL(DM x)
    %  SX = CEIL(SX x)
    %  MX = CEIL(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pl
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L530
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L530-L532
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(396, varargin{:});
    end
    function varargout = erf(varargin)
    %ERF Error function: x -> erf(x)
    %
    %  double = ERF(double x)
    %  DM = ERF(DM x)
    %  SX = ERF(SX x)
    %  MX = ERF(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pm
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L542
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L542-L544
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(397, varargin{:});
    end
    function varargout = erfinv(varargin)
    %ERFINV Inverse error function: x -> erfinv(x)
    %
    %  double = ERFINV(double x)
    %  DM = ERFINV(DM x)
    %  SX = ERFINV(SX x)
    %  MX = ERFINV(MX x)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pn
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L554
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L554-L556
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(398, varargin{:});
    end
    function varargout = sign(varargin)
    %SIGN Sign function:
    %
    %  double = SIGN(double x)
    %  DM = SIGN(DM x)
    %  SX = SIGN(SX x)
    %  MX = SIGN(MX x)
    %
    %
    %sign(x) := -1 for x<0 sign(x) := 1 for x>0, sign(0) := 0 sign(NaN) := 
    %NaN
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_po
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L571
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L571-L573
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(399, varargin{:});
    end
    function varargout = power(varargin)
    %POWER 
    %
    %  double = POWER(double x, double n)
    %  DM = POWER(DM x, DM n)
    %  SX = POWER(SX x, SX n)
    %  MX = POWER(MX x, MX n)
    %
    %
     [varargout{1:nargout}] = casadiMEX(400, varargin{:});
    end
    function varargout = rem(varargin)
    %REM Remainder after division: (x,y) -> fmod(x,y)
    %
    %  double = REM(double x, double y)
    %  DM = REM(DM x, DM y)
    %  SX = REM(SX x, SX y)
    %  MX = REM(MX x, MX y)
    %
    %
    %This Function follows the convention of 
    %https://en.cppreference.com/w/c/numeric/math/fmod
    %
    %Notably:
    %fmod(5,3) -> 2
    %
    %fmod(5,-3) -> 2
    %
    %fmod(-5,3) -> -2
    %
    %fmod(-5,-3) -> -2
    %
    %This is equivalent to Python's numpy.fmod and Matlab's rem.
    %
    %\\seealso remainder
    %
    %::
    %
    %  Extra doc: https://github.com/casadi/casadi/wiki/L_pq 
    %  
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L607
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L607-L609
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(401, varargin{:});
    end
    function varargout = fmod(varargin)
    %FMOD Remainder after division: (x,y) -> fmod(x,y)
    %
    %  double = FMOD(double x, double y)
    %  DM = FMOD(DM x, DM y)
    %  SX = FMOD(SX x, SX y)
    %  MX = FMOD(MX x, MX y)
    %
    %
    %This  Function follows the convention of 
    %https://en.cppreference.com/w/c/numeric/math/fmod
    %
    %Notably:
    %fmod(5,3) -> 2
    %
    %fmod(5,-3) -> 2
    %
    %fmod(-5,3) -> -2
    %
    %fmod(-5,-3) -> -2
    %
    %This is equivalent to Python's numpy.fmod and Matlab's rem.
    %
    %\\seealso remainder
    %
    %::
    %
    %  Extra doc: https://github.com/casadi/casadi/wiki/L_pq 
    %  
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L613
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L613-L615
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(402, varargin{:});
    end
    function varargout = remainder(varargin)
    %REMAINDER Remainder after division: (x,y) -> remainder(x,y)
    %
    %  double = REMAINDER(double x, double y)
    %  DM = REMAINDER(DM x, DM y)
    %  SX = REMAINDER(SX x, SX y)
    %  MX = REMAINDER(MX x, MX y)
    %
    %
    %This Function follows the convention of 
    %https://en.cppreference.com/w/c/numeric/math/remainder
    %
    %Notably:
    %remainder(5,3) -> -1
    %
    %remainder(5,-3) -> -1
    %
    %remainder(-5,3) -> 1
    %
    %remainder(-5,-3) -> 1
    %
    %This is equivalent to Python's math.remainder. There is no equivalence
    % in 
    %Matlab.
    %
    %\\seealso fmod
    %
    %::
    %
    %  Extra doc: https://github.com/casadi/casadi/wiki/L_24x 
    %  
    %
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L634
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L634-L636
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(403, varargin{:});
    end
    function varargout = atan2(varargin)
    %ATAN2 Two argument arc tangent: (y,x) -> atan2(y,x)
    %
    %  double = ATAN2(double x, double y)
    %  DM = ATAN2(DM x, DM y)
    %  SX = ATAN2(SX x, SX y)
    %  MX = ATAN2(MX x, MX y)
    %
    %
    %theta = atan2(y,x) corresponds to x = r cos(theta), y = r sin(theta)
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pr
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L648
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L648-L650
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(404, varargin{:});
    end
    function varargout = fmin(varargin)
    %FMIN Smallest of two values: (x,y) -> min(x,y)
    %
    %  double = FMIN(double x, double y)
    %  DM = FMIN(DM x, DM y)
    %  SX = FMIN(SX x, SX y)
    %  MX = FMIN(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pt
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L672
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L672-L674
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(405, varargin{:});
    end
    function varargout = fmax(varargin)
    %FMAX Largest of two values: (x,y) -> max(x,y)
    %
    %  double = FMAX(double x, double y)
    %  DM = FMAX(DM x, DM y)
    %  SX = FMAX(SX x, SX y)
    %  MX = FMAX(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pu
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L684
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L684-L686
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(406, varargin{:});
    end
    function varargout = hypot(varargin)
    %HYPOT Precision variant for 2 norm: (x,y) -> sqrt(x^2+y^2)
    %
    %  double = HYPOT(double x, double y)
    %  DM = HYPOT(DM x, DM y)
    %  SX = HYPOT(SX x, SX y)
    %  MX = HYPOT(MX x, MX y)
    %
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pw
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L742
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L742-L744
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(407, varargin{:});
    end
    function varargout = simplify(varargin)
    %SIMPLIFY 
    %
    %  double = SIMPLIFY(double x)
    %  DM = SIMPLIFY(DM x)
    %  SX = SIMPLIFY(SX x)
    %  MX = SIMPLIFY(MX x)
    %
    %
     [varargout{1:nargout}] = casadiMEX(408, varargin{:});
    end
    function varargout = is_equal(varargin)
    %IS_EQUAL Check if two nodes are equivalent up to a given depth.
    %
    %  bool = IS_EQUAL(double x, double y, int depth)
    %  bool = IS_EQUAL(DM x, DM y, int depth)
    %  bool = IS_EQUAL(SX x, SX y, int depth)
    %  bool = IS_EQUAL(MX x, MX y, int depth)
    %
    %
    %Depth=0 checks if the expressions are identical, i.e. points to the 
    %same 
    %node.
    %
    %a = x*x b = x*x
    %
    %is_equal(a,b,0) will return false, but a.is_equal(a,b,1) will return 
    %true
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_pv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L703
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L703-L705
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(409, varargin{:});
    end
    function varargout = copysign(varargin)
    %COPYSIGN Copy sign
    %
    %  double = COPYSIGN(double x, double y)
    %  DM = COPYSIGN(DM x, DM y)
    %  SX = COPYSIGN(SX x, SX y)
    %  MX = COPYSIGN(MX x, MX y)
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L710
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L710-L712
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(410, varargin{:});
    end
    function varargout = constpow(varargin)
    %CONSTPOW Elementwise power with const power
    %
    %  double = CONSTPOW(double x, double y)
    %  DM = CONSTPOW(DM x, DM y)
    %  SX = CONSTPOW(SX x, SX y)
    %  MX = CONSTPOW(MX x, MX y)
    %
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L720
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/generic_expression.hpp#L720-L722
    %
    %
    %
     [varargout{1:nargout}] = casadiMEX(411, varargin{:});
    end
    function self = GenericExpressionCommon(varargin)
    %GENERICEXPRESSIONCOMMON 
    %
    %  new_obj = GENERICEXPRESSIONCOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(412, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(413, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
