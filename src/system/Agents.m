% Agentsとは，Systemの中でも離散有限個の要素を持つもの
% 要素間に定義された距離を元にグラフ構造を持つ
classdef Agents < System
    properties
        Lap                 % グラフラプラシアン
        Adj                 % 接続行列
        Dist                % 距離行列
        Diff                % 差分行列
        P                   % 固有基底の入った正則行列
        Lambda              % 固有ベクトルを並べた対角行列
        Deg                 % 次数行列
    end
    properties %(Access = protected)
        dist_val            % 距離に関連する状態の集合
        rv                  % 距離の閾値
        is_weighted          % 重み付けグラフか否か
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
        end
        
        function obj = calcGraphMatrices(obj,t)
            obj = obj.calcDifferenceMatrix(t);
            obj = obj.calcEuclidDistanceMatrix;
            % obj.Adj = obj.Dist<obj.rv;
            obj.Adj = (obj.Dist<obj.rv)-eye(obj.N); % 08/07集成
            obj.Deg = diag(sum(obj.Adj,1));
            obj.Lap = obj.Deg-obj.Adj;
        end
        
        function obj = calcEignExpansion(obj,t)
            obj = obj.calcGraphMatrices(t);
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