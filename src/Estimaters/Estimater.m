classdef Estimater < System
    %ESTIMATER 観測器，推定機の大本．
    % systemの一種であることに間違いはない
    
    properties
        output_dim % 出力変数の次元
    end
    
    methods
        function obj = Estimater(N,output_dim,Nt,dt)
            %ESTIMATER このクラスのインスタンスを作成
            %   出力変数の次元なので注意
            obj@System(N,output_dim,Nt,dt);   % Systemクラスのコンストラクタ呼び出し
        end
        
        function obj = setParam(obj,inputArg)
            %METHOD1 パラメータ設定
            %   詳細説明をここに記述
            outputArg = obj.Property1 + inputArg;
        end
    end
end

