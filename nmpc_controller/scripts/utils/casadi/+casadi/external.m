function varargout = external(varargin)
    %EXTERNAL [INTERNAL] 
    %
    %  Function = EXTERNAL(char name, struct opts)
    %  Function = EXTERNAL(char name, Importer li, struct opts)
    %  Function = EXTERNAL(char name, char bin_name, struct opts)
    %
    %Load a just-in-time compiled external function.
    %
    %File name given
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_i2
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.hpp#L42
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.cpp#L42-L45
    %
    %
    %
    %.......
    %
    %::
    %
    %  EXTERNAL(char name, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Load an external function from a shared library.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name as in the label assigned to a CasADi  Function object: 
    %Function(name,...,...) Will be used to look up 
    %symbols/functions named eg. 
    %<name>_eval Use  nm (linux/osx) or  depends.exe (win) to check which symbols
    % are present in your shared library
    %
    %File name is assumed to be ./<name>.so
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_i0
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.hpp#L47
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.cpp#L47-L49
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
    %  EXTERNAL(char name, char bin_name, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Load an external function from a shared library.
    %
    %Parameters:
    %-----------
    %
    %name: 
    %Name as in the label assigned to a CasADi  Function object: 
    %Function(name,...,...) Will be used to look up 
    %symbols/functions named eg. 
    %<name>_eval Use  nm (linux/osx) or  depends.exe (win) to check which symbols
    % are present in your shared library
    %
    %bin_name: 
    %File name of the shared library
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_i1
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.hpp#L51
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.cpp#L51-L54
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
    %  EXTERNAL(char name, Importer li, struct opts)
    %
    %
    %
    %[INTERNAL] 
    %Load a just-in-time compiled external function.
    %
    %File name given
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_i2
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.hpp#L42
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/external.cpp#L42-L45
    %
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(803, varargin{:});
end
