
classdef myParameters_cheeger < matlab.mixin.SetGet
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
    end
    
    methods
        function obj = myParameters_cheeger()
            obj.Nt = 1000; % 500
            obj.dt = 0.01; % 0.02
            obj.rv = 0.6;
            obj.Omega_0 = 5;
            obj.kappa = 10;
            obj.gamma = 0;
            obj.tau = 20;
            obj.T = 1000;
            obj.I_0 = 0.001;%0.001;
            obj.num = 1;
            obj.omega_f = 3;
            %obj.formation = "agents20_rect_4_5";
            %obj.formation = "agents20_rect_2_10";
            %obj.formation = "agents21_ball";
            %obj.formation = "make_from_narrow";
            %obj.formation = "agents04_rect_1_4";
            obj.formation = "agents20_rect_1_20";
            obj.scheme = "explicit";%"state";
            obj.osc_IC = "zeros";%"random";
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