
classdef PikaPikaParameters < matlab.mixin.SetGet
    properties
        Nt      % シミュレーションカウント総数
        dt      % 刻み時間[s]
        rv
        Omega_0
        kappa
        r_s
        gamma
        T
        m
        d
        k_P
        k_F
        k_O
        d_O
        r_C
        I_0
        lambda
        T_f
        formation
    end
    
    methods
        function obj = PikaPikaParameters()
            obj.Nt = 3000;
            obj.dt = 0.05;
            obj.rv = 1.1;
            obj.Omega_0 = 0;
            obj.kappa = 0;
            obj.formation = "agents20_rect_4_5";
            obj.r_s = 0.6;
            obj.gamma = 1.0;
            obj.T = 0.05;
            obj.m = 1;
            obj.d = 1;
            obj.k_P = 0;
            obj.k_F = 1;
            obj.k_O = 0;
            obj.d_O = 1;
            obj.r_C = 1;
            obj.I_0 = 1;
            obj.lambda = 1;
            obj.T_f = 1;
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