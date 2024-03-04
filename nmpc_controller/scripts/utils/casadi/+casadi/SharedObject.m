classdef  SharedObject < SwigRef
    %SHAREDOBJECT [INTERNAL] 
    %
    %
    % SharedObject implements a reference counting framework similar 
    %for efficient and.
    %
    %easily-maintained memory management.
    %
    %To use the class, both the  SharedObject class (the public class), and the 
    %SharedObjectInternal class (the 
    %internal class) must be inherited from. It 
    %can be done in two 
    %different files and together with memory management, 
    %this approach 
    %provides a clear distinction of which methods of the class 
    %are to be 
    %considered "public", i.e. methods for public use that can be 
    %
    %considered to remain over time with small changes, and the internal 
    %memory.
    %
    %When interfacing a software, which typically includes including some 
    %header
    % file, this is best done only in the file where the internal 
    %class is 
    %defined, to avoid polluting the global namespace and other 
    %side effects.
    %
    %The default constructor always means creating a null pointer to an 
    %internal
    % class only. To allocate an internal class (this works only 
    %when the 
    %internal class isn't abstract), use the constructor with 
    %arguments.
    %
    %The copy constructor and the assignment operator perform shallow 
    %copies 
    %only, to make a deep copy you must use the clone method 
    %explicitly. This 
    %will give a shared pointer instance.
    %
    %In an inheritance hierarchy, you can cast down automatically, e.g. 
    %
    %(SXFunction is a child class of  Function): SXFunction derived(...);  
    %Function base = derived;
    %
    %To cast up, use the shared_cast template function, which works 
    %analogously 
    %to dynamic_cast, static_cast, const_cast etc, e.g.: 
    %SXFunction 
    %derived(...);  Function base = derived; SXFunction derived_from_base = 
    %
    %shared_cast<SXFunction>(base);
    %
    %A failed shared_cast will result in a null pointer (cf. dynamic_cast)
    %
    %Joel Andersson
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_as
    %
    %C++ includes: shared_object.hpp
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function varargout = class_name(self,varargin)
    %CLASS_NAME [INTERNAL] 
    %
    %  char = CLASS_NAME(self)
    %
    %Get class name.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_au
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L132
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L99-L101
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(23, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP [INTERNAL] 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %Print a description of the object.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L135
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L103-L109
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(24, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR [INTERNAL] 
    %
    %  char = STR(self, bool more)
    %
    %Get string representation.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L138
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L138-L142
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(25, self, varargin{:});
    end
    function varargout = print_ptr(self,varargin)
    %PRINT_PTR 
    %
    %  std::ostream & = PRINT_PTR(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(26, self, varargin{:});
    end
    function varargout = is_null(self,varargin)
    %IS_NULL [INTERNAL] 
    %
    %  bool = IS_NULL(self)
    %
    %Is a null pointer?
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L150
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L73-L75
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(27, self, varargin{:});
    end
    function varargout = hash(self,varargin)
    %HASH [INTERNAL] 
    %
    %  int = HASH(self)
    %
    %Returns a number that is unique for a given Node.
    %
    %If the Object does not point to any node, "0" is returned.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_av
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.hpp#L157
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/shared_object.cpp#L129-L131
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(28, self, varargin{:});
    end
    function self = SharedObject(varargin)
    %SHAREDOBJECT 
    %
    %  new_obj = SHAREDOBJECT()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(29, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(30, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
