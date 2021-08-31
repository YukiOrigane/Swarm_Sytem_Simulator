
% 結合振動子系に対するLF(Leader-Follower)制御．
% リーダーでのみ固有周波数を変える
classdef COS_PikaPika_Controller
    properties
        u_cos           % 制御入力 
        Leader_num = 1  % リーダー番号
        
        Kappa           % 結合強度（台数×台数）の対角行列．次元ごとに違うと大変
        Omega_0         % 共通固有周波数 (1×振動子次元横ベクトル)
        Omega_L         % リーダー固有周波数 (1×振動子次元横ベクトル)
        I_0
        lambda
        T_f
        input_type
    end
    
    methods
        % コンストラクタ
        function obj = COS_PikaPika_Controller
        end
        
        % 入力の計算
        function obj = calcInput(obj,t,sys)   % システムを引数にとる
            f = zeros(sys.N,1);
            if obj.input_type == "PeriodicPurturbation"
                f = obj.I_0 * sin(2*pi/obj.lambda.*sys.sys_robot.x(:,1,t))*sin(2*pi/obj.T_f*t*sys.dt);
            end
            omega = repmat(obj.Omega_0, sys.N, 1) + f;
            %phi_gap_sum = permute( sum( repmat(sys.Adj,1,1,sys.dim).*sys.Diff, 2), [1,3,2]); % エージェント数×次元の行列になる
            obj.u_cos = reshape(omega - (obj.Kappa./ diag(sys.sys_robot.Deg)) .* (sys.sys_cos.Lap * sys.sys_cos.x(:,1,t)) , sys.N*sys.sys_cos.dim, 1);
        end
        
        % パラメータセット．共通固有周波数,リーダー固有周波数,結合強度（対角行列 or 共通スカラ）
        function obj = setParam(obj,I_0, Omega_0, kappa)
            obj.I_0 = I_0;
            obj.Omega_0 = Omega_0;
            obj.Kappa = kappa;
        end
        
        function obj = setPeriodicPurturbation(obj, lambda, T_f)
            obj.input_type = "PeriodicPurturbation";
            obj.lambda = lambda;
            obj.T_f = T_f;
        end
        
    end
end