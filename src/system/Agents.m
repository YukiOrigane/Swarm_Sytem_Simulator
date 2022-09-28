% Agentsとは，Systemの中でも離散有限個の要素を持つもの
% 要素間に定義された距離を元にグラフ構造を持つ
classdef Agents < System
    properties
        Lap                 % グラフラプラシアン
        Lap_norm            % 正規化グラフラプラシアン
        Adj                 % 接続行列
        Dist                % 距離行列
        Diff                % 差分行列
        P                   % 固有基底の入った正則行列
        Lambda              % 固有ベクトルを並べた対角行列
        Deg                 % 次数行列
        degree              % 総エッジ本数
        xi                  % モード座標
        Nabla               % 偏微分行列
    end

    properties %(Access = protected)
        dist_val            % 距離に関連する状態の集合
        rv                  % 距離の閾値
        is_weighted          % 重み付けグラフか否か
        is_calc_mode_coordinate % モード座標の計算をするか否か
    end
    
    methods
        function obj = Agents(N,dim,Nt,dt) % コンストラクタ
            obj@System(N,dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し
        end
        
        function obj = observe(obj,t)     % 現在の状態から色々作る
            obj = obj.calcGraphMatrices(t);
        end
        
        function obj = update(obj,t,u) % 状態更新，入力 u を引く
           % 処理 
        end
        
        function obj = calcModeCoordinate(obj,t)    % モード座標の計算．xのアップデート後に実施すること
            if ~obj.is_calc_mode_coordinate 
                return % モード座標計算しないことになってたら帰る
            end
            obj = obj.calcEigenExpansionFromLap(t); % グラフラプラシアンを複数回計算している可能性あり．注意
            for d = 1:obj.dim
                obj.xi(:,d,t) = obj.P.'*obj.x(:,d,t);   % \xi = P^T x
            end
        end

        function obj = setCalcModeCoordinate(obj)   % モード座標計算を有効化．速度もメモリも食うので注意
            obj.is_calc_mode_coordinate = true;
            obj.xi = zeros(size(obj.x)+[0 0 1]);    % モード座標をxと同じ大きさで初期化
        end

        function obj = setGraphProperties(obj,dist_val,rv,is_weightd)
            obj.dist_val = dist_val;
            obj.rv = rv;
            if ~isempty(is_weightd)
                obj.is_weighted = is_weightd;
            else
                obj.is_weighted = false;
            end
            obj.Diff = zeros(obj.N,obj.N,length(dist_val));
            obj.Dist = zeros(obj.N);
            obj.Nabla = zeros(obj.N,obj.N,length(dist_val));
        end
        
        function obj = calcGraphMatrices(obj,t,type)
            arguments
                obj
                t
                type {mustBeMember(type,{'dist1','dist2','dist3','counting'})} = 'counting';
            end
            obj = obj.calcDifferenceMatrix(t);
            obj = obj.calcEuclidDistanceMatrix;
            % obj.Adj = obj.Dist<obj.rv;
            if strcmp(type,'dist1')
                obj.Adj = 1./(obj.Dist+eye(obj.N))-eye(obj.N);  % w_ij = 1/||x_ij||
            elseif strcmp(type,'dist2')
                obj.Adj = 1./(obj.Dist.^2+eye(obj.N))-eye(obj.N);  % w_ij = 1/||x_ij||^2
            elseif strcmp(type,'dist3')
                obj.Adj = 1./(obj.Dist.^3+eye(obj.N))-eye(obj.N);  % w_ij = 1/||x_ij||^2
            else
                obj.Adj = (obj.Dist<obj.rv)-eye(obj.N); % 08/07集成
            end
            obj.Deg = diag(sum(obj.Adj,1));
            obj.Lap = obj.Deg-obj.Adj;
            obj.Lap_norm = inv(obj.Deg)*obj.Lap;  % 正規化の計算
            %obj.degree = sum(obj.Deg,'all');
        end
        
        function obj = calcNablaMatrix(obj,t)    % 偏微分行列の計算
            % obj = obj.calcGparhMatrices(obj,t);   % 計算しといてね
            for dim = obj.dist_val
                Adj_nabla = obj.Adj .* obj.Diff(:,:,dim)./(obj.Dist+eye(obj.N)); % 隣接とのcos\theta_{ij}が入る
                Deg_nabla = diag(sum(Adj_nabla,2));
                obj.Nabla(:,:,dim) = Adj_nabla - Deg_nabla; % % \sum_{j\in\mathcal{N}_i} (\phi_j-\phi_i)\cos\theta_{ij}
                % Lapとは適用する符号が逆になっているので注意
                % cosが重み行列だと思ってしまうと，2乗してほしくなる->2乗すれば各方向のラプラシアンそのものとなる
            end
        end

        function obj = calcEignExpansion(obj,t)
            obj = obj.calcGraphMatrices(t);
            [obj.P,obj.Lambda] = eig(obj.Lap);
            [lambda,ind] = sort(diag(obj.Lambda));
            obj.Lambda = obj.Lambda(ind,ind);
            obj.P = obj.P(ind,ind);
        end

        function obj = calcEigenExpansionFromLap(obj,t) % Lが与えられている場合（Lを再計算しない）
            [obj.P,obj.Lambda] = eig(obj.Lap);
            [lambda,ind] = sort(diag(obj.Lambda));
            obj.Lambda = obj.Lambda(ind,ind);
            obj.P = obj.P(ind,ind);
        end
        
        function obj = calcEuclidDistanceMatrix(obj)
            obj.Dist = vecnorm(obj.Diff,2,3);
        end
        
        function obj = calcDifferenceMatrix(obj,t)
            for dim = obj.dist_val
                obj.Diff(:,:,dim) = repmat(obj.x(:,dim,t),1,obj.N)-repmat(obj.x(:,dim,t).',obj.N,1);
            end
        end
        
    end % methods
end % Agents