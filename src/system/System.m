% Systemとは，同等の複数要素を持ちうる，時間発展するもの
classdef System
    properties
        x                   % state variables 
        N                   % number of element
        dim {mustBeInteger} % dimension of state
        dt                  % time step
        Nt                  % simulation count
        dim_name_list       % name of dimension
        t_vec;
        subsystems         % list of subsystem
        u_histry           % 入力の時間履歴
    end
    methods
        function obj = System(N,dim,Nt,dt)
            % コンストラクタ
            obj.N = N;
            obj.dim = dim;
            obj.dt = dt;
            obj.Nt = Nt;
            obj.x = zeros(N,dim,Nt);
            obj.u_histry = zeros(N,dim,Nt);
            obj.t_vec = 0:dt:Nt*dt;
        end
        function obj = observe(obj)
            % 現在の状態から制御器に渡す何かの計算
            
            % サブシステムがある場合はそれの計算も
            %for subsystem = obj.subsystems
            %    obj.subsystem = obj.subsystem.observe;
            %end
        end
        function obj = update(obj, t, u)
            % 時間発展 update process
            % 引数は入力 u
            
            % サブシステムがある場合はそれの計算も
            %for subsystem = obj.subsystems
            %    obj.subsystem = obj.subsystem.update(obj,t,u);
            %end
            obj.u_histry(:,:,t) = u;
        end
        function obj = setInitialCondition(obj,x0)
            obj.x(:,:,1) = x0;
        end
        function obj = setDimName(obj,dim_name_list)
            obj.dim_name_list = dim_name_list;
        end
        
         % 制限範囲を超える状態量があったら反対側に飛ばす処理
         % limit_list = [min_x1, max_x1; min_x2, max_x2;...; min_xn, max_xn]
        function obj = areaConstraintJump(obj,t,limit_list)
            n_fixed = size(limit_list, 1);
            range = repmat( (limit_list(:,2)-limit_list(:,1)).', obj.N, 1 );    % 空間の幅を取得
            obj.x(:,1:n_fixed,t) = obj.x(:,1:n_fixed,t) + range.*(obj.x(:,1:n_fixed,t)<repmat(limit_list(:,1).',obj.N,1)) - range.*(obj.x(:,1:n_fixed,t)>repmat(limit_list(:,2).',obj.N,1));
            
        end
        
    end
end