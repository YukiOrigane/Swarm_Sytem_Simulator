
classdef CBF < Controller
    properties
        is_active   % 有効？
        w           % 重み係数 (expected to 2)
        u_nom
        type        % 種類
        r_s
        gamma
        T
        h
        Lfh
        Lgh
    end
    
    methods
        function obj = CBF(is_activate)
            obj = obj.setActive(is_activate);
            obj.w = 2;
        end
        function obj = setActive(obj,is_active)
            obj.is_active = is_active;
        end
        
        function obj = calcInput(obj,t,sys,u_nom)
            obj.u = zeros(size(u_nom));
            if obj.is_active ~= "true"
                return
            end
            obj.u_nom = reshape(u_nom,sys.N,[]);
            u = zeros(size(obj.u_nom));
            obj.Lfh = zeros(sys.N,sys.N,1); % 台数×台数×1
            obj.Lgh = zeros(sys.N,sys.N,2,1);
            obj.h = zeros(sys.N,sys.N,2,1);
            if obj.type == "SecondOrderLinear"
                obj = obj.calcSecondOrderLinearCBF(t,sys);
            end
            A = obj.w*permute(obj.Lgh(:,:,:), [2,3,1]);
            b = permute(obj.Lfh(:,:,1) + obj.gamma*obj.h(:,:,1), [2,3,1]);
            bar_u = zeros(size(obj.u_nom));
            for i = 1:sys.N
                options = optimoptions('quadprog','Display','none');
                [bar_u(i,:),fval,exitflag] = quadprog(eye(2), [], -A(:,:,i), b(:,:,i)-A(:,:,i)*obj.u_nom(i,:).',[],[],[],[],[0,0],options);
                if exitflag ~= 1
                   bar_u(i,:) = [0,0];
                end
            end
            obj.u = reshape(obj.u_nom - bar_u(:,:),[],1);
        end
        
        function obj = setParam(obj, w) % 重み変更
            obj.w = w;
        end
        
        function obj = setSecondOrderLinear(obj, r_s, gamma, T)
            obj.type = "SecondOrderLinear";
            obj.r_s = r_s;
            obj.gamma = gamma;
            obj.T = T;
        end
        
        function obj = calcSecondOrderLinearCBF(obj,t,sys)
            q1x = repmat(sys.x(:,1,t),1,sys.N)-repmat(sys.x(:,1,t).',sys.N,1);
            q1y = repmat(sys.x(:,2,t),1,sys.N)-repmat(sys.x(:,2,t).',sys.N,1);
            q2x = repmat(sys.x(:,3,t),1,sys.N)-repmat(sys.x(:,3,t).',sys.N,1);
            q2y = repmat(sys.x(:,4,t),1,sys.N)-repmat(sys.x(:,4,t).',sys.N,1);
            qnorm = sqrt(q1x.^2+q1y.^2);
            for i = 1:sys.N
                for j = 1:sys.N
                    if (sys.Adj(i,j)~=1)
                        continue
                    end
                    nabla = [q1x(i,j)+q1y(i,j)*(q2x(i,j)*q1y(i,j)-q2y(i,j)*q1x(i,j))/(qnorm(i,j)^2)*obj.T;
                        q1y(i,j)+q1x(i,j)*(q2y(i,j)*q1x(i,j)-q2x(i,j)*q1y(i,j))/(qnorm(i,j)^2)*obj.T;
                        q1x(i,j)*obj.T;
                        q1y(i,j)*obj.T];
                    f = [q2x(i,j); q2y(i,j); -sys.d/sys.m*q2x(i,j); -sys.d/sys.m*q2y(i,j)];
                    g = [0 0; 0 0; 1/sys.m 0; 0 1/sys.m];
                    obj.Lfh(i,j,1) = nabla.'*f;
                    obj.Lgh(i,j,1) = nabla.'*g(:,1); obj.Lgh(i,j,2) = nabla.'*g(:,2);
                    obj.h(i,j,1) = qnorm(i,j) + [q2x(i,j), q2y(i,j)]*[q1x(i,j); q1y(i,j)]/qnorm(i,j)*obj.T-obj.r_s;
                end
            end
        end
    end
end