
% 注意：多分時間の単位がmsになる

classdef ChlamydomonasController < Controller
    properties  % please see a paper
        I_0
        v_0
        omega_c_0
        omega_a_0
        gamma_0
        tau_0_ms
        interior_sys % pとIを入れる
    end
    
    methods
        
        % コンストラクタ
        function obj = ChlamydomonasController(N,Nt,dt)
            obj.interior_sys = Agents(N,8,Nt,dt); % I, p, eyespot(x,y,z)
            obj.interior_sys = obj.interior_sys.setInitialCondition([zeros(N,8)]);
        end
        
        function obj = setParam(obj,I_0,v_0,omega_c_0, omega_a_0, gamma_0, tau_0_ms)
            %obj.a = [1 0 0];          % 相対ベクトル基底
            %obj.b = [0 1 0];
            %obj.c = [0 0 1];
            obj.I_0 = I_0;
            obj.v_0 = v_0;
            obj.omega_c_0 = omega_c_0;
            obj.omega_a_0 = omega_a_0;
            obj.gamma_0 = gamma_0;
            obj.tau_0_ms = tau_0_ms;
        end
        
        function obj = calcInput(obj,t,sys)   % システムを引数にとる
            if sys.dim == 6
                Rot = eul2rotm(sys.x(:,4:6,t)); % 回転行列の計算．びっくりするほど便利 [3,3,エージェント数]
                Rot = pagetranspose(Rot);
            elseif sys.dim == 7
                Rot = quat2rotm(sys.x(:,4:7,t));
                %Rot = pagetranspose(Rot);
            end
            a = pagemtimes( repmat([1 0 0],1,1,sys.N), Rot); % 各エージェントの局所基底 [1,3,エージェント数]
            b = pagemtimes( repmat([0 1 0],1,1,sys.N), Rot);
            c = pagemtimes( repmat([0 0 1],1,1,sys.N), Rot);
            e_light = repmat([0 0 -1],1,1,sys.N);
            e_eyespot = (b+c)/sqrt(2);
            obj.interior_sys.x(:,3:5,t+1) = permute(e_eyespot,[3,2,1]);
            obj.interior_sys.x(:,6:8,t+1) = permute(c,[3,2,1]);
            obj.interior_sys.x(:,1,t+1) = obj.I_0 * (permute( pagemtimes(-e_light,'none',e_eyespot,'transpose'), [3,1,2])) /2; % [エージェント数,1,1]
            
            obj.u = zeros(sys.N,sys.dim); % 入力は0で初期化
            obj.u(:,1:3) = obj.v_0 * permute( c, [3,2,1] );
            tau = obj.tau_0_ms * 10^-3 / sys.dt;
            if t > tau % 時間遅れ対応可能？
                obj.interior_sys.x(:,2,t+1) = obj.gamma_0 * (obj.interior_sys.x(:,1,t-tau)-obj.interior_sys.x(:,1,t-tau+1))/sys.dt;
                %obj.u(:,1:3) = obj.v_0 * permute( c, [3,2,1] );
                obj.u(:,4:6) = -obj.omega_c_0 * permute( c, [3,2,1] ) + (obj.omega_a_0-obj.interior_sys.x(:,2,t)).* permute( a, [3,2,1] );
            end
        end
        
    end
end