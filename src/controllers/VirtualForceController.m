
classdef VirtualForceController < Controller
    properties
        % see 折金のSymmetry 2021
        k_P     % 位相勾配力係数
        k_F     % 群形成力係数
        k_O     % 衝突回避力係数
        d_O     % 衝突回避距離
        r_C     % 平衡距離
    end
    
    methods
        function obj = calcInput(obj, t, sys)
            f_F = zeros(sys.N,2);
            f_O = zeros(sys.N,2);
            f_P = zeros(sys.N,2);
            %f_iO = 
            Dist_p = eye(sys.N) + sys.sys_robot.Dist./obj.r_C;
            f_F(:,1) = -obj.k_F * sum(sys.sys_robot.Adj.*((Dist_p.^(-3)-Dist_p^(-2)).*exp(Dist_p).*sys.sys_robot.Diff(:,:,1)),2);
            f_F(:,2) = -obj.k_F * sum(sys.sys_robot.Adj.*((Dist_p.^(-3)-Dist_p^(-2)).*exp(Dist_p).*sys.sys_robot.Diff(:,:,2)),2);
            sys.sys_cos.dist_val = 1;
            sys.sys_cos = sys.sys_cos.calcDifferenceMatrix(t);
            F_P(:,1) = diag(sys.sys_cos.Diff*(sys.sys_robot.Diff(:,:,1)./(eye(sys.N) + sys.sys_robot.Dist)).').';
            F_P(:,2) = diag(sys.sys_cos.Diff*(sys.sys_robot.Diff(:,:,2)./(eye(sys.N) + sys.sys_robot.Dist)).').';
            f_P = obj.k_P * F_P./(vecnorm(F_P,2,2)+(vecnorm(F_P,2,2)==0));
            %obj.u = [zeros(sys.N*sys.sys_robot.dim/2, 1) ; reshape(f_F + f_O + f_P, sys.N*sys.sys_robot.dim/2, 1)];
            obj.u = [reshape(f_F + f_O + f_P, sys.N*sys.sys_robot.dim/2, 1)];
        end
        
        function obj = setParam(obj, k_P, k_F, k_O, d_O, r_C)
            obj.k_P = k_P;
            obj.k_F = k_F;
            obj.k_O = k_O;
            obj.d_O - d_O;
            obj.r_C = r_C;
        end
    end
    
end