
% 角速度と並進速度を与えられた場合の3次元システム
% 変数は空間固定での点x,y,z及びZYZオイラー角

classdef EularAngle1stOrderSystem < Agents
    properties
        
    end
    
    methods
        function obj = EularAngle1stOrderSystem(N,dim,Nt,dt) % コンストラクタ
            if dim < 6
                disp("要求次元が6以下でした．6次元に自動的に変換します")
                dim = 6;
            end
            obj@Agents(N,dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し      
        end
        
        % u : [エージェント数,6]
        function obj = update(obj,t,u)
            wx = u(:,4); wy = u(:,5); wz = u(:,6);
            th1 = obj.x(:,4,t); th2 = obj.x(:,5,t); th3 = obj.x(:,6,t);
            obj.x(:,1:3,t+1) = obj.x(:,1:3,t) + obj.dt * u(:,1:3); % 並進入力はuの1列から3列．そのまま反映される
            %obj.x(:,4,t+1) = obj.x(:,4,t) + obj.dt * csc(th2).*(-wx.*cos(th3)+wy.*sin(th3));
            %obj.x(:,5,t+1) = obj.x(:,5,t) + obj.dt * ( wx.*sin(th3)+wy.*cos(th3));
            %obj.x(:,6,t+1) = obj.x(:,6,t) + obj.dt * ( wz-cot(th2).*(-wx.*cos(th3)+wy.*sin(th3)));
            
            obj.x(:,6,t+1) = obj.x(:,6,t) + obj.dt * csc(th2).*(wx.*cos(th1)+wy.*sin(th1));
            obj.x(:,5,t+1) = obj.x(:,5,t) + obj.dt * ( -wx.*sin(th1)+wy.*cos(th1));
            obj.x(:,4,t+1) = obj.x(:,4,t) + obj.dt * ( wz-cot(th2).*(wx.*cos(th1)+wy.*sin(th1)));
        end
    end
end