classdef RevoluteJoint < iDynTree.MovableJointImpl1
  methods
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(971, self);
        self.swigPtr=[];
      end
    end
    function varargout = clone(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(972, self, varargin{:});
    end
    function varargout = setAttachedLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(973, self, varargin{:});
    end
    function varargout = setRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(974, self, varargin{:});
    end
    function varargout = setAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(975, self, varargin{:});
    end
    function varargout = getFirstAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(976, self, varargin{:});
    end
    function varargout = getSecondAttachedLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(977, self, varargin{:});
    end
    function varargout = getAxis(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(978, self, varargin{:});
    end
    function varargout = getRestTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(979, self, varargin{:});
    end
    function varargout = getTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(980, self, varargin{:});
    end
    function varargout = getTransformDerivative(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(981, self, varargin{:});
    end
    function varargout = getMotionSubspaceVector(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(982, self, varargin{:});
    end
    function varargout = computeChildPosVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(983, self, varargin{:});
    end
    function varargout = computeChildVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(984, self, varargin{:});
    end
    function varargout = computeChildVelAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(985, self, varargin{:});
    end
    function varargout = computeChildAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(986, self, varargin{:});
    end
    function varargout = computeChildBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(987, self, varargin{:});
    end
    function varargout = computeJointTorque(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(988, self, varargin{:});
    end
    function varargout = hasPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(989, self, varargin{:});
    end
    function varargout = enablePosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(990, self, varargin{:});
    end
    function varargout = getPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(991, self, varargin{:});
    end
    function varargout = getMinPosLimit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(992, self, varargin{:});
    end
    function varargout = getMaxPosLimit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(993, self, varargin{:});
    end
    function varargout = setPosLimits(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(994, self, varargin{:});
    end
    function self = RevoluteJoint(varargin)
      self@iDynTree.MovableJointImpl1(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        error('No matching constructor');
      end
    end
  end
  methods(Static)
  end
end