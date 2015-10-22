classdef LinearForceVector3 < iDynTree.ForceVector3__LinearForceVector3
  methods
    function self = LinearForceVector3(varargin)
      self@iDynTree.ForceVector3__LinearForceVector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(308, varargin{:});
        tmp = iDynTreeMATLAB_wrap(308, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(309, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
