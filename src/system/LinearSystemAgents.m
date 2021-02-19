% 線形システムを持つエージェント群

classdef LinearSystemAgents < Agents
    properties% (Access = protected)
        A_11 % x(t) to dxdt(t+1) : (Na*dim)×(Na*dim)行列
        A_12 % dxdt(t) to dxdt(t+1) : (Na*dim)×(Na*dim)行列
        A_21 % x(t) to d^2xdt^2 : (Na*dim)×(Na*dim)行列
        A_22 % dxdt(t) to d^2xdt^2 : (Na*dim)×(Na*dim)行列
        B_1 % u to dxdt(t+1) : (Na*dim)×(uの次元)行列
        B_2 % u to d^2xdt^2 : (Na*dim)×(uの次元)行列
    end
    methods
        
        function obj = LinearSystemAgents(N,dim,Nt,dt) % コンストラクタ
            obj@Agents(N,dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し      
        end
        
        function obj = update(obj,t,u)
            x = reshape(obj.x(:,:,t),obj.N*obj.dim,1);
            dxdt = reshape(obj.dxdt(:,:,t),obj.N*obj.dim,1);   % 縦ベクトルに変換
            obj.x(:,:,t+1) = obj.x(:,:,t) + obj.dt .* reshape(obj.A_11*x+obj.A_12*dxdt+obj.B_1*u, obj.N, obj.dim);
            obj.dxdt(:,:,t+1) = obj.dxdt(:,:,t) + obj.dt .* reshape(obj.A_21*x+obj.A_22*dxdt+obj.B_2*u, obj.N, obj.dim);
        end
    end
end