
% 結合振動子系に対するLF(Leader-Follower)制御．
% リーダーでのみ固有周波数を変える
classdef COS_LF_Controller
    properties
        u_cos           % 制御入力 
        Leader_num = 1  % リーダー番号
        
        Kappa           % 結合強度（台数×台数）の対角行列．次元ごとに違うと大変
        Omega_0         % 共通固有周波数 (1×振動子次元横ベクトル)
        Omega_L         % リーダー固有周波数 (1×振動子次元横ベクトル)
    end
    
    methods
        % コンストラクタ
        function obj = COS_LF_Controller
        end
        
        % 入力の計算
        function obj = calcInput(obj,t,sys)   % システムを引数にとる
            omega = repmat(obj.Omega_0, sys.N, 1);
            omega(obj.Leader_num, :) = obj.Omega_L;
            sys = sys.setGraphProperties(1:sys.dim,0,false);
            sys = sys.calcDifferenceMatrix(t);
            phi_gap_sum = permute( sum( repmat(sys.Adj,1,1,sys.dim).*sys.Diff, 2), [1,3,2]); % エージェント数×次元の行列になる
            obj.u_cos = reshape(omega - (obj.Kappa.*eye(sys.N)) * phi_gap_sum, sys.N*sys.dim, 1);
        end
        
        % パラメータセット．共通固有周波数,リーダー固有周波数,結合強度（対角行列 or 共通スカラ）
        function obj = setParam(obj,Omega_0, Omega_L, kappa)
            obj.Omega_0 = Omega_0;
            obj.Omega_L = Omega_L;
            obj.Kappa = kappa;
        end
        
        % リーダー番号の設定
        function obj = setLeaderNum(obj,num)
            obj.Leader_num = num;
        end
    end
end