
% GFTに基づくSDOFに対して線形FBでどうなるか
classdef SDOF_Controller <  LinearFeedBackController
    methods
        function obj = calcInput(obj,t,sys,ref)
            obj = obj.setReference(ref);
            sysNa = sys.N; sysdim = sys.dim;
            sys = sys.calcEignExpansion(t); % GFT計算
            Pd = [];
            for d=1:sysdim/2     % 座標変換行列を対角ブロックに並べる
                Pd = blkdiag(Pd,sys.P);
            end
            obj.u = Pd*obj.Gain*Pd.'*(reshape(sys.P*obj.ref-sys.x(:,1:sys.dim/2,t), sysNa*sysdim/2, 1));
            obj.u = [zeros(length(obj.u),1); obj.u];
        end
    end
end