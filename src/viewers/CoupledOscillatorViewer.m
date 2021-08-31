
classdef CoupledOscillatorViewer < PlaneBasicViewer
    
    methods
        function obj = CoupledOscillatorViewer(sys,field)
            obj@PlaneBasicViewer(sys,field);
        end
        
        function phaseGapPlot(obj,set,base_num) % 表示エージェント集合,基準エージェント番号
            %plot(obj.sys.t_vec, permute(obj.sys.x(set,obj.sys.dim,:)-obj.sys.x(base_num,obj.sys.dim,:),[1,3,2]));
            plot(obj.sys.t_vec, permute(obj.sys.x(set,1,:)-obj.sys.x(base_num,1,:),[1,3,2]));
            lgd = legend(string(set));
            lgd.String(base_num) = cellstr(string(base_num)+": (基準)");
            lgd.NumColumns = round(length(set)/4);
        end
        
        % 共通角速度分を引く
        function phasePhiPlot(obj,set,Omega_0)
            %plot(obj.sys.t_vec, permute(obj.sys.x(set,1,:),[1,3,2])-mod(obj.sys.t_vec*Omega_0,2*pi));
            plot(obj.sys.t_vec, permute(obj.sys.x(set,1,:),[1,3,2])-obj.sys.t_vec*Omega_0);
            lgd = legend(string(set));
            lgd.NumColumns = round(length(set)/4);
        end
        
        function virtualEnergyVIewer(obj)
            obj.energyTimePlot(2);
        end
    end
end