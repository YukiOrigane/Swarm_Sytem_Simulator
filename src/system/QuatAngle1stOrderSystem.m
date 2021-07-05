
% 角速度と並進速度を与えられた場合の3次元システム
% 変数は空間固定での点x,y,z及びZYZオイラー角

classdef QuatAngle1stOrderSystem < Agents
    properties
        
    end
    
    methods
        function obj = QuatAngle1stOrderSystem(N,dim,Nt,dt) % コンストラクタ
            if dim < 7
                disp("要求次元が7以下でした．7次元に自動的に変換します")
                dim = 7;
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
            p = permute(wx, [2,3,1]); q = permute(wy, [2,3,1]); r = permute(wz, [2,3,1]);
            A = zeros(4,4,obj.N);
            %A = [0 r -q p; -r 0 p q; q -0 0 r; -p -q -r 0];
            A = [0 -p -q -r; p 0 r -q; q -r 0 p; r q -p 0];
            obj.x(:,4:7,t+1) = obj.x(:,4:7,t) + obj.dt * 1/2*permute( pagemtimes(A,permute(obj.x(:,4:7,t),[2,3,1])), [3,1,2]);
        end
    end
end