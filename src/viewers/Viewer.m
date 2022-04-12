
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
        
        function inputTimePlotLinear(obj,n,dim) % 入力時刻歴プロット
            plot(obj.sys.t_vec(1:obj.sys.Nt), permute(obj.sys.u_histry(n,dim,:),[1,3,2]));
        end
        
        function inputSpectrumPlotLinear(obj,n,dim) % 入力パワープロット
            pspectrum(permute(obj.sys.u_histry(n,dim,1:obj.sys.Nt),[1,3,2]),obj.sys.t_vec(1:obj.sys.Nt));
        end
        
        function energyTimePlot(obj,dim)
            plot(obj.sys.t_vec, permute(sum(obj.sys.x(:,dim,:).^2,1)/2*obj.sys.dt,[1,3,2]));
        end

        function energyTimePlot2(obj,energy_dim)
            if energy_dim == obj.sys.energy_dim+1   % 要求された次元がエネルギーの次元より1だけ大きい場合は総和を返す
                plot(obj.sys.t_vec(2:end-1), sum(obj.sys.energy(:,2:end-1),1),'LineWidth',5);
                return
            end
            plot(obj.sys.t_vec, obj.sys.energy(energy_dim,:));
        end
    end
end