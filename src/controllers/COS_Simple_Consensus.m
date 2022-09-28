
% 結合振動子系に対する単純なコンセンサス制御
classdef COS_Simple_Consensus
    properties
        u_cos           % 制御入力 
        Input
        Leader_num = 1  % リーダー番号
        f % 外部入力（[エージェント数の縦ベクトル]）
        tau
        T
        I_0
        num
        Kappa           % 結合強度（台数×台数）の対角行列．次元ごとに違うと大変
        Omega_0         % 共通固有周波数 (1×振動子次元横ベクトル)
    end
    
    methods
        % コンストラクタ
        function obj = COS_Simple_Consensus
            obj.Input = "none";
        end
        
        % 入力の計算
        function obj = calcInput(obj,t,sys)   % システムを引数にとる
            switch obj.Input
                case "Impulse"
                    obj = obj.updateInpulseInput(t,sys);
                case "WhiteNoise"
                    obj = obj.updateWhiteNoise(t,sys);
                case "PeriodicInput"
                    obj = obj.updatePeriodicInput(t,sys);
                otherwise
            end
            omega = repmat(obj.Omega_0, sys.N, 1);
            sys = sys.setGraphProperties(1:sys.dim,0,false);
            sys = sys.calcDifferenceMatrix(t);
            phi_gap_sum = permute( sum( repmat(sys.Adj,1,1,sys.dim).*sin(sys.Diff), 2), [1,3,2]); % エージェント数×次元の行列になる
            obj.u_cos = reshape(omega - (obj.Kappa) * phi_gap_sum + obj.f, sys.N*sys.dim, 1);
        end
        
        % パラメータセット．共通固有周波数,リーダー固有周波数,結合強度（対角行列 or 共通スカラ）
        function obj = setParam(obj,Omega_0, kappa)
            obj.Omega_0 = Omega_0;
            obj.Kappa = kappa;
        end

        % リーダー番号の設定
        function obj = setLeaderNum(obj,num)
            obj.Leader_num = num;
        end

        % インパルス入力をセット 事前にインパルス入力を仕組む
        function obj = setInpulseInput(obj,tau,T,I_0,num)
            obj.tau = tau;  % スタート時刻
            obj.T = T;      % 入力長さ
            obj.I_0 = I_0;  % 強度
            obj.num = num;  % 入力エージェント
            obj.Input = "Impulse";
        end
        
        % インパルス入力を計算 システム，開始時刻，長さ，強度，対象エージェントナンバー
        function obj = updateInpulseInput(obj,t,sys)
            obj.f = zeros(sys.N,1);
            if (t>obj.tau)&&(t<obj.tau+obj.T)
                obj.f(obj.num) = obj.I_0;
            end
        end
        
    end
end