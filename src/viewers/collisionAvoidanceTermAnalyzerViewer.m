
% 衝突回避項の解析と表示
classdef collisionAvoidanceTermAnalyzerViewer < PlaneModeViewer
    properties
        Wavoidance % 回避重み行列
        Phi        % 衝突回避ポテンシャル
        f_CA       % 衝突回避力（エージェント空間）
        f_hat_CA   % 衝突回避力（固有空間） (mode, dim)
        dist_dim   % 距離の関連する状態の次元数
        U          % 空間のポテンシャル
        dUdx       % ポテンシャルのx方向勾配
        dUdy       % ポテンシャルのy方向勾配
        
    end
    
    methods
        function obj = collisionAvoidanceTermAnalyzerViewer(sys,field)
            obj@PlaneModeViewer(sys,field);
            obj.dist_dim = length(sys.dist_val);
            obj.Wavoidance = zeros(sys.N, sys.N, obj.dist_dim);
            obj.Phi = zeros(sys.N, sys.N);
            obj.f_CA = zeros(sys.N, obj.dist_dim);
            obj.f_hat_CA = zeros(sys.N, obj.dist_dim);
        end
        function obj = calcCollisionAvoindaceTerm(obj,t)
            obj = obj.analyzeGraphMode(t);
            for d = 1:obj.dist_dim
                obj.Wavoidance(:,:,d) = obj.sys.Diff(:,:,d)./((obj.sys.Dist+eye(obj.sys.N)).^3); % 回避重み行列の計算．クラスにして分ける？
                obj.f_CA(:,d) = diag(-obj.sys.Adj * obj.Wavoidance(:,:,d));
            end
            obj.Phi(:,:) = 1./((obj.sys.Dist+eye(obj.sys.N)).^2);
            obj.f_hat_CA = obj.V.' * obj.f_CA;
        end
        
        function obj = calcPotential(obj,t,dx,dy,sigma)
            obj = obj.setPlaneGrid(dx,dy);
            Mid = zeros(obj.sys.N,obj.sys.N,2);
            for dim = [1,2]
                Mid(:,:,dim) = repmat(obj.sys.x(:,dim,t),1,obj.sys.N)+repmat(obj.sys.x(:,dim,t).',obj.sys.N,1)./2;  % 中点座標行列
            end
            nv = 1/2*obj.sys.N*(obj.sys.N-1);
            xmid = zeros(1,nv);
            ymid = zeros(1,nv);
            v = zeros(1,nv);
            cnt = 1;
            for n = 1:obj.sys.N-1 % 対角成分の引っ張り出し
               xmid(cnt:cnt+obj.sys.N-n-1) = diag(Mid(:,:,1),n);
               ymid(cnt:cnt+obj.sys.N-n-1) = diag(Mid(:,:,2),n);
               v(cnt:cnt+obj.sys.N-n-1) = diag(obj.sys.Adj(:,:),n).*diag(obj.Phi(:,:),n);
               cnt = cnt + obj.sys.N-1;
            end
            obj.U = zeros(length(obj.lx)*length(obj.lx),1);
            for k = 1:nv
                obj.U = obj.U + mvnpdf([obj.xq(:) obj.yq(:)], [xmid(k) ymid(k)], sigma*eye(2)); % 多変量ガウス分布の足し合わせ : 本当はポテンシャル関数の形に合わせるべきでは？
            end
            obj.U = reshape(obj.U,length(obj.lx),length(obj.ly));
            u = [1 0 -1]';
            v = [1 2 1];
            obj.dUdx = conv2(v,u,obj.U);
            obj.dUdy = conv2(u,v,obj.U);
        end
        
        function plotPotential(obj)
            imagesc(obj.lx,obj.ly,obj.U);
            set(gca,'FontSize',12);
            title("Potential map");
        end
        
        function plotGradient(obj,dim)
            if dim == 1
                imagesc(obj.lx,obj.ly,obj.dUdx);
                title("Potential Gradient (x) map");
            else
                imagesc(obj.lx,obj.ly,obj.dUdy);
                title("Potential Gradient (y) map");
            end
            set(gca,'FontSize',12);
        end
        
        function plotForceVector(obj,t,mode_list,layout,type)
            cnt = 0;
            for mode = mode_list
                cnt = cnt+1;
                subplot(layout(1),layout(2),cnt);
                if type == "Position"
                    obj.plotPosition(t,obj.V(:,mode));
                elseif type == "Mesh"
                    obj.plotMesh(t,obj.V(:,mode));
                end
                %obj.plot3d(1,obj.V(:,mode));
                %obj.plotScatter3d(1,obj.V(:,mode),obj.V(:,mode));
                hold on
                xg = sum(obj.sys.x(:,1,t))./obj.sys.N;
                yg = sum(obj.sys.x(:,2,t))./obj.sys.N;
                quiver(xg,yg, obj.f_hat_CA(mode,1), obj.f_hat_CA(mode,2),'LineWidth',6,'MaxHeadSize',0.6);
                title(string(mode)+" th mode ; \lambda_{"+string(mode)+"} = " +string(obj.d(mode)));
            end
        end
        
        function plotForceAbsolute(obj)
            plot(vecnorm(obj.f_hat_CA(:,:),2,2),'*');
            title("Collision Avoidance Force $f_{\lambda_i}$",'Interpreter','latex');
            xlabel("i th mode");
            ylabel("force")
            set(gca,'Fontsize',12);
            grid on
        end
        
        function plotForceAbsoluteVsFreq(obj)
            plot(sqrt(abs(obj.d)), vecnorm(obj.f_hat_CA(:,:),2,2),'*');
            title("Collision Avoidance Force $f_{\lambda_i}$",'Interpreter','latex');
            xlabel("Spatial Frequency $\sqrt{\lambda}$",'Interpreter','latex');
            ylabel("force")
            set(gca,'Fontsize',12);
            grid on
        end
    end
end