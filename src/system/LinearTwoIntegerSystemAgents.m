classdef LinearTwoIntegerSystemAgents < LinearSystemAgents
    properties %(Access = protected)
        spatial_dim % spacial dimension. dim = 2*spatial_dimension. 
        A_11 % x(t) to dxdt(t+1) : (Na*sp_dim)×(Na*sp_dim)行列
        A_12 % dxdt(t) to dxdt(t+1) : (Na*sp_dim)×(Na*sp_dim)行列
        A_21 % x(t) to d^2xdt^2 : (Na*sp_dim)×(Na*sp_dim)行列
        A_22 % dxdt(t) to d^2xdt^2 : (Na*sp_dim)×(Na*sp_dim)行列
        B_1 % u to dxdt(t+1) : (Na*sp_dim)×(uの次元)行列
        B_2 % u to d^2xdt^2 : (Na*sp_dim)×(uの次元)行列
    end
    
    methods
        function obj = LinearTwoIntegerSystemAgents(N,spatial_dim,Nt,dt) % コンストラクタ
            obj@LinearSystemAgents(N,2*spatial_dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し
            obj.spatial_dim = spatial_dim;
            % spatial_dim で空間自由度を定義．2階積分系なので状態変数の次元は空間自由度の2倍
        end
        
        % 各m,d,kは(要素数)×(次元)の行列．各要素の質量，ダンパ，ばね定数をそれぞれ指定
        % 入力ベクトルの次元は状態ベクトルの次元と一致する．
        function obj = setMechanicalParameters(obj,m,d,k)
            obj.A_11 = zeros(obj.N*obj.spatial_dim);
            obj.A_12 = eye(obj.N*obj.spatial_dim);
            obj.B_1 = zeros(obj.N*obj.spatial_dim);
            M = diag(reshape(m,obj.N*obj.spatial_dim,1));
            D = diag(reshape(d,obj.N*obj.spatial_dim,1));
            K = diag(reshape(k,obj.N*obj.spatial_dim,1));
            obj.A_21 = -K.*pinv(M);
            obj.A_22 = -D.*pinv(M);
            obj.B_2 = 1.*pinv(M);
            obj.A = [obj.A_11, obj.A_12; obj.A_21, obj.A_22];
            obj.B = [obj.B_1,zeros(obj.N*obj.spatial_dim) ;obj.B_2, obj.B_2];
        end
    end
end