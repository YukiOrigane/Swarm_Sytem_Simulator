
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
        scheme
        osc_IC  % 振動子位相の初期条件 zero -> 0, random -> [0,2pi]乱数
        is_normalize
    end
    
    methods
        function obj = Parameters()
            obj.Nt = 2000;
            obj.dt = 0.02;
            obj.rv = 2.1;
            obj.Omega_0 = 0;
            obj.kappa = 0.01;
            obj.gamma = 1;
            obj.tau = 100;
            obj.T = 2000;
            obj.I_0 = 1;%0.001;
            obj.num = 1;
            obj.omega_f = 3;
            %obj.formation = "agents03_rect_1_3";
            obj.formation = "agents04_rect_1_4";
            %obj.formation = "agents04_rect_2_2";
            %obj.formation = "agents10_rect_1_10";
            obj.scheme = "explicit";%"state";
            obj.osc_IC = "zeros";
            obj.is_normalize = false;%true;
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