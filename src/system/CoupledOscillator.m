
% 結合振動子系
classdef CoupledOscillator < Agents
    properties
        is_2pi_periodic
    end
    
    methods
        function obj = CoupledOscillator(N,dim,Nt,dt) % コンストラクタ
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
            obj.x(:,:,t+1) = obj.x(:,:,t) + obj.dt .* reshape(u_cos, obj.N, obj.dim);
        end
    end
end