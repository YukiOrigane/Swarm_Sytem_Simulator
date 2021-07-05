
classdef Viewer
    properties
        sys                 % 描画する対象のシステム
        field
        param
    end
    
    methods
        function obj = Viewer(sys,field)
            obj.sys = sys;
            obj.field = field;
        end
        
        function obj = setParamList(obj,para)
            obj.param = para;
        end
        
        function view(obj)  % 表示する関数 : オーバーライド前提
        end
        function basicTimePlotX(obj,dim) % 何かの次元のタイムプロット
            plot(obj.sys.t_vec, permute(obj.sys.x(:,dim,:),[1,3,2]));
        end
    end
end