% Systemとは，同等の複数要素を持ちうる，時間発展するもの
classdef System
    properties
        x                   % state variables 
        dxdt                % time deriverative of variables
        N                   % number of element
        dim {mustBeInteger} % dimension of state
        dt                  % time step
        Nt                  % simulation count
        dim_name_list       % name of dimension
        t_vec;
    end
    methods
        function obj = System(N,dim,Nt,dt)
            % コンストラクタ
            obj.N = N;
            obj.dim = dim;
            obj.dt = dt;
            obj.Nt = Nt;
            obj.x = zeros(N,dim,Nt);
            obj.dxdt = zeros(N,dim,Nt);
            obj.t_vec = 0:dt:Nt*dt;
        end
        function obj = observe(obj)
            % 現在の状態から制御器に渡す何かの計算
        end
        function obj = update(obj, t, u)
            % 時間発展 update process
            % 引数は入力 u
        end
        function obj = setInitialCondition(obj,x0,dxdt0)
            obj.x(:,:,1) = x0;
            obj.dxdt(:,:,1) = dxdt0;
        end
        function obj = setDimName(obj,dim_name_list)
            obj.dim_name_list = dim_name_list;
        end
    end
end