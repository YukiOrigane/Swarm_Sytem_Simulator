
% 超シンプルな1階積分系システム
% A行列もB行列も単位行列．入力でよしなにする
% dxdt = x + u

classdef SimpleOneIntegerSystemAgents < LinearSystemAgents
    methods
        function obj = SimpleOneIntegerSystemAgents(N,dim,Nt,dt) % コンストラクタ
            obj@LinearSystemAgents(N,dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し
            
            obj.A = zeros(obj.N*obj.dim);
            obj.B = eye(obj.N*obj.dim);
        end
    end
end