
classdef Viewer
    properties
        sys                 % 描画する対象のシステム
        field
    end
    
    methods
        function obj = Viewer(sys,field)
            obj.sys = sys;
            obj.field = field;
        end
        function view(obj)  % 表示する関数 : オーバーライド前提
        end
        function basicTimePlotX(obj,dim) % 何かの次元のタイムプロット
            plot(obj.sys.t_vec, permute(swarm.x(:,dim,:),[1,3,2]));
        end
    end
end