
% 結合振動子系
classdef COS_Second_Order_Controller < COS_LF_Controller
    properties
        gamma   % 固有周波数の更新粘性
        f % 外部入力（[エージェント数の縦ベクトル]）
        Input
        tau
        T
        I_0
        num
        omega_f
    end
    
    methods
        % コンストラクタ
        function obj = COS_Second_Order_Controller
            obj.T = 0;  % 通常はインパルス入らない
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
            %sys = sys.setGraphProperties(1:sys.dim,1.2,false);
            %sys = sys.calcDifferenceMatrix(t);
            %phi_gap_sum = permute( sum( repmat(sys.Adj,1,1,sys.dim).*sin(sys.Diff), 2), [1,3,2]); % エージェント数×次元の行列になる
            if isempty(obj.f)
                obj.f = zeros(sys.N,1);
            end
            %obj.u_cos = [omega+sys.x(:,2,t) , reshape( -obj.gamma*sys.x(:,2,t) - (obj.Kappa) * phi_gap_sum(:,1) + obj.f, sys.N, 1)];
            %sys = sys.calcGraphMatrices(t);
            obj.u_cos = [omega+sys.x(:,2,t) , reshape( -obj.gamma*(sys.x(:,2,t)-omega) - (obj.Kappa) * sys.Lap *sys.x(:,1,t)  + obj.f, sys.N, 1)];
        end
        
        % パラメータセット．共通固有周波数,リーダー固有周波数,結合強度（対角行列 or 共通スカラ）
        function obj = setParam(obj,Omega_0, kappa, gamma)
            obj.Omega_0 = Omega_0;
            obj.Kappa = kappa;
            obj.gamma = gamma;
        end
        
        % インパルス入力をセット 事前にインパルス入力を仕組む
        function obj = setInpulseInput(obj,tau,T,I_0,num)
            obj.tau = tau;
            obj.T = T;
            obj.I_0 = I_0;
            obj.num = num;
            obj.Input = "Impulse";
        end
        
        % インパルス入力を計算 システム，開始時刻，長さ，強度，対象エージェントナンバー
        function obj = updateInpulseInput(obj,t,sys)
            obj.f = zeros(sys.N,1);
            if (t>obj.tau)&&(t<obj.tau+obj.T)
                obj.f(obj.num) = obj.I_0;
            end
        end
        
        % ホワイトノイズをセット
        function obj = setWhiteNoise(obj,tau,T,I_0,num)
            obj.tau = tau;
            obj.T = T;
            obj.I_0 = I_0;
            obj.num = num;
            obj.Input = "WhiteNoise";
        end
        
        % ホワイトノイズを注入
        function obj = updateWhiteNoise(obj,t,sys)
            obj.f = zeros(sys.N,1);
            if (t>obj.tau)&&(t<obj.tau+obj.T)
                obj.f(obj.num) = obj.I_0*(rand-0.5);
            end
        end
        
        % 正弦波をセット
        function obj = setPeriodicInput(obj,tau,T,I_0,num,omega_f)
            obj.tau = tau;
            obj.T = T;
            obj.I_0 = I_0;
            obj.num = num;
            obj.omega_f = omega_f;
            obj.Input = "PeriodicInput";
        end
        
         % 正弦波を注入
        function obj = updatePeriodicInput(obj,t,sys)
            obj.f = zeros(sys.N,1);
            if (t>obj.tau)&&(t<obj.tau+obj.T)
                obj.f(obj.num) = obj.I_0*sin(obj.omega_f*t*sys.dt);
            end
        end
        
    end
end