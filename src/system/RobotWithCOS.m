
% 振動子を搭載したロボット群システム．
% ロボットは線形二階積分系と思っておく
classdef RobotWithCOS < Agents
    properties
        sp_dim          % ロボットの移動する空間の次元
        oscillator_dim  % 振動子の次元
        sys_robot       % ロボットシステム          
        sys_cos         % COSシステム
    end
    
    methods
        % コンストラクタ．エージェント数,空間次元,振動子次元,シミュレーション数,刻み時間
        function obj = RobotWithCOS(N,sp_dim,osc_dim,Nt,dt)
            obj@Agents(N,sp_dim+osc_dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し
            obj.sys_robot = LinearTwoIntegerSystemAgents(N,sp_dim,Nt,dt);   % 移動体のシステム宣言
            obj.sys_cos = CoupledOscillator(N,osc_dim,Nt,dt);
        end
        
        % グラフラプラシアン周りの諸設定
        function obj = setGraphProperties(obj,dist_val,rv,is_weightd)
            obj.sys_robot = obj.sys_robot.setGraphProperties(dist_val,rv,is_weightd);
        end
        
        % グラフ構造は位置で決める．それを他のシステムにも流す
        function obj = observe(obj,t)
            obj.sys_robot = obj.sys_robot.calcGraphMatrices(t);
            obj.Adj = obj.sys_robot.Adj;
            obj.Lap = obj.sys_robot.Lap;
            obj.sys_cos.Adj = obj.Adj;
            obj.sys_cos.Lap = obj.Lap;
        end
        
        % 入力決定．時刻,移動入力,振動子入力,自然角周波数
        function obj = update(obj,t,u_robot, u_cos)
            obj.sys_robot = obj.sys_robot.update(t,u_robot);
            obj.sys_cos = obj.sys_cos.update(t,u_cos);
        end
    end
end