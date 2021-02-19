classdef LinearTwoIntegerSystemAgents < LinearSystemAgents
    properties (Access = protected)
    end
    
    methods
        function obj = LinearTwoIntegerSystemAgents(N,dim,Nt,dt) % コンストラクタ
            obj@LinearSystemAgents(N,dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し      
        end
        
        % 各m,d,kは(要素数)×(次元)の行列．各要素の質量，ダンパ，ばね定数をそれぞれ指定
        % 入力ベクトルの次元は状態ベクトルの次元と一致する．
        function obj = setMechanicalParameters(obj,m,d,k)
            obj.A_11 = zeros(obj.N*obj.dim);
            obj.A_12 = eye(obj.N*obj.dim);
            obj.B_1 = zeros(obj.N*obj.dim);
            M = diag(reshape(m,obj.N*obj.dim,1));
            D = diag(reshape(d,obj.N*obj.dim,1));
            K = diag(reshape(k,obj.N*obj.dim,1));
            obj.A_21 = -K.*pinv(M);
            obj.A_22 = -D.*pinv(M);
            obj.B_2 = 1.*pinv(M);
        end
    end
end