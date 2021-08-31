
% 結合振動子系
classdef CoupledOscillatorExplicit < Agents
    properties
        is_2pi_periodic
    end
    
    methods
        function obj = CoupledOscillatorExplicit(N,dim,Nt,dt) % コンストラクタ
            obj@Agents(N,dim,Nt,dt);   % Agentsクラスのコンストラクタ呼び出し
            obj.is_2pi_periodic = false;
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
    end
end