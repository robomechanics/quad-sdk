classdef  PrintableCommon < SwigRef
    %PRINTABLECOMMON 
    %
    %   = PRINTABLECOMMON()
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end

    function s = repr(self)
      s = [self.type_name() '(' self.str() ')'];
    end
      function self = PrintableCommon(varargin)
    %PRINTABLECOMMON 
    %
    %  new_obj = PRINTABLECOMMON()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(21, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(22, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
