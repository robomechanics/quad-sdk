classdef  Importer < casadi.SharedObject & casadi.PrintableCommon
    %IMPORTER [INTERNAL] 
    %
    %
    % Importer.
    %
    %Just-in-time compilation of code
    %General informationList of plugins
    %- clang
    %
    %- shell
    %
    %Note: some of the plugins in this list might not be available on your 
    %
    %system.  Also, there might be extra plugins available to you that are 
    %not 
    %listed here. You can obtain their documentation with   
    %Importer.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %clang
    %-----
    %
    %
    %
    %Interface to the JIT compiler CLANG
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_21v
    %
    %>List of available options
    %
    %+--------------+-----------------+-----------------------------------------+
    %|      Id      |      Type       |               Description               |
    %+==============+=================+=========================================+
    %| flags        | OT_STRINGVECTOR | Compile flags for the JIT compiler.     |
    %|              |                 | Default: None                           |
    %+--------------+-----------------+-----------------------------------------+
    %| include_path | OT_STRING       | Include paths for the JIT compiler. The |
    %|              |                 | include directory shipped with CasADi   |
    %|              |                 | will be automatically appended.         |
    %+--------------+-----------------+-----------------------------------------+
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %shell
    %-----
    %
    %
    %
    %Interface to the JIT compiler SHELL
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_22w
    %
    %>List of available options
    %
    %+----------------------+-----------------+---------------------------------+
    %|          Id          |      Type       |           Description           |
    %+======================+=================+=================================+
    %| cleanup              | OT_BOOL         | Cleanup temporary files when    |
    %|                      |                 | unloading. Default: true        |
    %+----------------------+-----------------+---------------------------------+
    %| compiler             | OT_STRING       | Compiler command                |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_flags       | OT_STRINGVECTOR | Alias for 'compiler_flags'      |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_output_flag | OT_STRING       | Compiler flag to denote object  |
    %|                      |                 | output. Default: '-o '          |
    %+----------------------+-----------------+---------------------------------+
    %| compiler_setup       | OT_STRING       | Compiler setup command.         |
    %|                      |                 | Intended to be fixed. The       |
    %|                      |                 | 'flag' option is the prefered   |
    %|                      |                 | way to set custom flags.        |
    %+----------------------+-----------------+---------------------------------+
    %| directory            | OT_STRING       | Directory to put temporary      |
    %|                      |                 | objects in. Must end with a     |
    %|                      |                 | file separator.                 |
    %+----------------------+-----------------+---------------------------------+
    %| extra_suffixes       | OT_STRINGVECTOR | List of suffixes for extra      |
    %|                      |                 | files that the compiler may     |
    %|                      |                 | generate. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| flags                | OT_STRINGVECTOR | Compile flags for the JIT       |
    %|                      |                 | compiler. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| linker               | OT_STRING       | Linker command                  |
    %+----------------------+-----------------+---------------------------------+
    %| linker_flags         | OT_STRINGVECTOR | Linker flags for the JIT        |
    %|                      |                 | compiler. Default: None         |
    %+----------------------+-----------------+---------------------------------+
    %| linker_output_flag   | OT_STRING       | Linker flag to denote shared    |
    %|                      |                 | library output. Default: '-o '  |
    %+----------------------+-----------------+---------------------------------+
    %| linker_setup         | OT_STRING       | Linker setup command. Intended  |
    %|                      |                 | to be fixed. The 'flag' option  |
    %|                      |                 | is the prefered way to set      |
    %|                      |                 | custom flags.                   |
    %+----------------------+-----------------+---------------------------------+
    %| name                 | OT_STRING       | The file name used to write out |
    %|                      |                 | compiled objects/libraries. The |
    %|                      |                 | actual file names used depend   |
    %|                      |                 | on 'temp_suffix' and include    |
    %|                      |                 | extensions. Default:            |
    %|                      |                 | 'tmp_casadi_compiler_shell'     |
    %+----------------------+-----------------+---------------------------------+
    %| temp_suffix          | OT_BOOL         | Use a temporary (seemingly      |
    %|                      |                 | random) filename suffix for     |
    %|                      |                 | file names. This is desired for |
    %|                      |                 | thread-safety. This behaviour   |
    %|                      |                 | may defeat caching compiler     |
    %|                      |                 | wrappers. Default: true         |
    %+----------------------+-----------------+---------------------------------+
    %
    %Joris Gillis
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_161
    %
    %C++ includes: importer.hpp
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
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L118
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L71-L73
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(912, self, varargin{:});
    end
    function varargout = has_function(self,varargin)
    %HAS_FUNCTION [INTERNAL] 
    %
    %  bool = HAS_FUNCTION(self, char symname)
    %
    %
      [varargout{1:nargout}] = casadiMEX(913, self, varargin{:});
    end
    function varargout = has_meta(self,varargin)
    %HAS_META [INTERNAL] 
    %
    %  bool = HAS_META(self, char cmd, int ind)
    %
    %Does a meta entry exist?
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_165
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L145
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L83-L85
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(914, self, varargin{:});
    end
    function varargout = get_meta(self,varargin)
    %GET_META [INTERNAL] 
    %
    %  char = GET_META(self, char cmd, int ind)
    %
    %Get entry as a text.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_166
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L150
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L87-L89
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(915, self, varargin{:});
    end
    function varargout = inlined(self,varargin)
    %INLINED [INTERNAL] 
    %
    %  bool = INLINED(self, char symname)
    %
    %Check if a function is inlined.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L153
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L91-L93
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(916, self, varargin{:});
    end
    function varargout = body(self,varargin)
    %BODY [INTERNAL] 
    %
    %  char = BODY(self, char symname)
    %
    %Get the function body, if inlined.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L156
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L95-L97
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(917, self, varargin{:});
    end
    function varargout = library(self,varargin)
    %LIBRARY [INTERNAL] 
    %
    %  char = LIBRARY(self)
    %
    %Get library name.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L159
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L99-L101
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(918, self, varargin{:});
    end
    function varargout = serialize(self,varargin)
    %SERIALIZE [INTERNAL] 
    %
    %  SERIALIZE(self, casadi::SerializingStream & s)
    %
    %Serialize an object.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_16c
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L211
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L103-L105
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(919, self, varargin{:});
    end
    function self = Importer(varargin)
    %IMPORTER 
    %
    %  new_obj = IMPORTER()
    %  new_obj = IMPORTER(char name, char compiler, struct opts)
    %
    %
    %.......
    %
    %::
    %
    %  IMPORTER()
    %
    %
    %
    %[INTERNAL] 
    %Default constructor.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L94
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L31-L32
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
    %  IMPORTER(char name, char compiler, struct opts)
    %
    %
    %
    %[INTERNAL] 
    % Importer factory.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.hpp#L97
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/importer.cpp#L34-L45
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
        tmp = casadiMEX(921, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(922, self);
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
     [varargout{1:nargout}] = casadiMEX(907, varargin{:});
    end
    function varargout = test_cast(varargin)
    %TEST_CAST 
    %
    %  bool = TEST_CAST(casadi::SharedObjectInternal const * ptr)
    %
    %
     [varargout{1:nargout}] = casadiMEX(908, varargin{:});
    end
    function varargout = has_plugin(varargin)
    %HAS_PLUGIN 
    %
    %  bool = HAS_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(909, varargin{:});
    end
    function varargout = load_plugin(varargin)
    %LOAD_PLUGIN 
    %
    %  LOAD_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(910, varargin{:});
    end
    function varargout = doc(varargin)
    %DOC 
    %
    %  char = DOC(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(911, varargin{:});
    end
    function varargout = deserialize(varargin)
    %DESERIALIZE 
    %
    %  Importer = DESERIALIZE(casadi::DeserializingStream & s)
    %
    %
     [varargout{1:nargout}] = casadiMEX(920, varargin{:});
    end
  end
end
