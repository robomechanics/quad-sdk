classdef  CodeGenerator < SwigRef
    %CODEGENERATOR [INTERNAL] 
    %
    %
    %Helper class for C code generation.
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_ru
    %
    %C++ includes: code_generator.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = CodeGenerator(varargin)
    %CODEGENERATOR [INTERNAL] 
    %
    %  new_obj = CODEGENERATOR()
    %
    %Constructor.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.hpp#L46
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.cpp#L35-L180
    %
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(887, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = add(self,varargin)
    %ADD [INTERNAL] 
    %
    %  ADD(self, Function f, bool with_jac_sparsity)
    %
    %Add a function (name generated)
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.hpp#L49
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.cpp#L288-L316
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(888, self, varargin{:});
    end
    function varargout = dump(self,varargin)
    %DUMP [INTERNAL] 
    %
    %  char = DUMP(self)
    %
    %Generate a file, return code as string.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.hpp#L57
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.cpp#L318-L322
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(889, self, varargin{:});
    end
    function varargout = generate(self,varargin)
    %GENERATE [INTERNAL] 
    %
    %  char = GENERATE(self, char prefix)
    %
    %Generate file(s)
    %
    %The "prefix" argument will be prepended to the generated files and 
    %may be
    % a directory or a file prefix. returns the filename
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_rv
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.hpp#L66
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.cpp#L410-L464
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(890, self, varargin{:});
    end
    function varargout = add_include(self,varargin)
    %ADD_INCLUDE [INTERNAL] 
    %
    %  ADD_INCLUDE(self, char new_include, bool relative_path, char use_ifdef)
    %
    %Add an include file optionally using a relative path "..." 
    %instead 
    %of an absolute path <...>
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.hpp#L69
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/code_generator.cpp#L942-L962
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(891, self, varargin{:});
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(892, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
