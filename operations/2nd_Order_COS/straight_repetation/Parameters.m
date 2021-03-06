
classdef Parameters < matlab.mixin.SetGet
    properties
        Nt      % シミュレーションカウント総数
        dt      % 刻み時間[s]
        rv
        Omega_0
        kappa
        gamma
        tau
        T
        I_0
        num
        omega_f
        formation
    end
    
    methods
        function obj = Parameters()
            obj.Nt = 600;
            obj.dt = 0.01;
            obj.rv = 1.5;
            obj.Omega_0 = 5;
            obj.kappa = 100;
            obj.gamma = 0.1;
            obj.tau = 100;
            obj.T = 1000;
            obj.I_0 = 0.001;
            obj.num = 1;
            obj.omega_f = 3;
            obj.formation = "agents10_rect_1_10";
        end
        
        function obj = setParamList(obj,mat)
            if size(mat,2) ~= 2
                disp("[WARNING] length of parameters are not proper")
                return
            end
            len = size(mat,1);
            for i = 1:len
                obj = setParam(mat(i,1),mat(i,2));
            end
        end
        
        function obj = setParam(obj,prop,val)
            if isprop(obj,prop)
                set(obj,prop,val);
            else
                disp("[WARNING] "+prop+"is not property name")
            end
        end
    end
end