
% 結合振動子系
classdef CoupledOscillatorExplicit < Agents
    properties
        is_2pi_periodic
    end
    
    methods
        function obj = CoupledOscillatorExplicit(N,dim,Nt,dt) % コンストラクタ
            obj@Agents(N,dim,Nt,dt);   % Agentsクラスのコンストラクタ呼び出し
            obj.is_2pi_periodic = false;
            obj = obj.setEnergyProperties(2);   % 疑似運動エネと疑似ポテンシャルエネ
        end
        
        function obj = setPeriodic(obj,flag)
            obj.is_2pi_periodic = flag;
        end
        
        function obj = update(obj,t,u_cos)
            if obj.is_2pi_periodic
                obj.x(:,1,t) = mod(obj.x(:,1,t),2*pi);
            end
            %obj.x(:,:,t+1) = obj.x(:,:,t) + obj.dt .* reshape(u_cos, obj.N, obj.dim);
            obj.x(:,1,t+1) = reshape(u_cos, obj.N, 1);
            if (t>=2)
                obj.x(:,2,t+1) = (obj.x(:,1,t+1)-obj.x(:,1,t-1))/2/obj.dt;
            else
                obj.x(:,2,t+1) = obj.x(:,2,t);
            end
            %obj.u_histry(:,2,t) = reshape(u_cos, obj.N, 1);
        end

        function obj = calcEnergy(obj,t,kappa)
            if obj.energy_dim == 0  % 次元設定がなされてないなら計算しない
                return
            end
            obj.energy(1,t) = 1/2*obj.x(:,2,t+1).'*obj.x(:,2,t+1)*obj.dt;  % 疑似運動エネルギー
            obj.energy(2,t) = 1/2*kappa*obj.x(:,1,t).'*obj.Lap*obj.x(:,1,t)*obj.dt; % 疑似ポテンシャルエネルギー
            %Lall = [2 -1 -1 0; -1 2 0 -1; -1 0 2 -1; 0 -1 -1 2];
            %Lall = [3 -1 -1 -1; -1 3 -1 -1; -1 -1 3 -1; -1 -1 -1 3];
            %obj.energy(2,t) = 1/2*kappa*obj.x(:,1,t).'*Lall*obj.x(:,1,t)*obj.dt; % 疑似ポテンシャルエネルギー
        end

    end
end