classdef Rotation < iDynTree.RotationRaw
  methods
    function self = Rotation(varargin)
      self@iDynTree.RotationRaw(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(742, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(743, self, varargin{:});
    end
    function varargout = changeOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(744, self, varargin{:});
    end
    function varargout = changeRefOrientFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(745, self, varargin{:});
    end
    function varargout = changeCoordinateFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(746, self, varargin{:});
    end
    function varargout = changeCoordFrameOf(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(749, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(750, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(751, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(752, self, varargin{:});
    end
    function varargout = fromQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(753, self, varargin{:});
    end
    function varargout = getRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(754, self, varargin{:});
    end
    function varargout = asRPY(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(755, self, varargin{:});
    end
    function varargout = getQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(756, self, varargin{:});
    end
    function varargout = asQuaternion(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(757, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(770, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(771, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(772, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(747, varargin{:});
    end
    function varargout = inverse2(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(748, varargin{:});
    end
    function varargout = RotX(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(758, varargin{:});
    end
    function varargout = RotY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(759, varargin{:});
    end
    function varargout = RotZ(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(760, varargin{:});
    end
    function varargout = RotAxis(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(761, varargin{:});
    end
    function varargout = RotAxisDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(762, varargin{:});
    end
    function varargout = RPY(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(763, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(764, varargin{:});
    end
    function varargout = RPYRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(765, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivative(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(766, varargin{:});
    end
    function varargout = QuaternionRightTrivializedDerivativeInverse(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(767, varargin{:});
    end
    function varargout = Identity(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(768, varargin{:});
    end
    function varargout = RotationFromQuaternion(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(769, varargin{:});
    end
  end
end
