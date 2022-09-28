classdef geometricalInformationEstimater < Estimater
    %GEOMETRICALINFORMATIONESTIMATER 周波数解析を使った空間情報推定
    %   ピーク推定をごにょごにょやる
    
    properties
        t_with % FFT観測幅
        power_threshold % パワー閾値
        input_sys RobotWithCOS
        partial_phi % 空間微分値の格納
        N_band = 4096
        peak_power % ピークのパワー強度 <- zに組み込むべきという説もある．
        peak_locs
    end
    
    methods
        function obj = geometricalInformationEstimater(N,sp_dim,Nt,dt)
            %GEOMETRICALINFORMATIONESTIMATER 台数,空間次元数,時間ステップ数,ステップ幅
            %   詳細説明をここに記述
            obj@Estimater(N,sp_dim*2,Nt,dt);   % 正直次元何に使うは微妙．
            % 出力次元：台数推定値(x,y), 内外判定(x,y)
            obj.partial_phi = zeros(N, sp_dim, Nt);
            obj.peak_power = zeros(N,2*sp_dim,Nt);
            obj.power_threshold = 10^-10;   % すれしょるど初期値
        end

        function obj = observeAll(obj, sys)   
            % 事後的に全部突っ込んで，最終時刻における値のみ計算
            obj = obj.observe(Nt,sys);
        end

        function obj = observe(obj, t, sys)
            % 観測量の計算
            obj.input_sys = sys;
            sp_dim = obj.input_sys.sys_robot.spatial_dim;
            obj = obj.spatialDifferentiation(t);    % 空間偏微分の計算

            if t<=obj.t_with
                %t_index_list = 1:t; % 観測範囲を定義
                return;
            else
                t_index_list = t-obj.t_with:t;
            end
            t_vec_part = obj.t_vec(t_index_list);
            phi_part = zeros(obj.N,sp_dim+1,length(t_index_list)); % FFTに使う位相量の格納
            phi_part(:,1,:) = obj.input_sys.sys_cos.x(:,1,t_index_list);    % 位相そのまま
            phi_part(:,2:3,:) = obj.partial_phi(:,:,t_index_list);  % 空間偏微分それぞれ
            phi_powers  = zeros(obj.N,sp_dim+1,obj.N_band);
            obj.peak_locs = zeros(obj.N,sp_dim+1);
            peak_freqs = zeros(obj.N,sp_dim+1);

            
            for dim = 1:sp_dim+1
                [p,f] = pspectrum(permute(phi_part(:,dim,:),[3,1,2]),t_vec_part); % パワースペクトラム解析(各次元毎一括)
                phi_powers(:,dim,:) = permute(p,[2,3,1]);
            end

            for i = 1:obj.N
                locations = zeros(sp_dim+1,obj.N_band);  % ピーク位置を格納する配列
                for dim = 1:sp_dim+1
                    [pks,locs] = findpeaks(permute(phi_powers(i,dim,:),[3,1,2]),"MinPeakHeight",obj.power_threshold);
                    locations(dim, :) = padarray(locs, obj.N_band-length(locs), obj.N_band,'post');    % パディングしながらピーク位置を格納
                end
                obj.peak_locs(i,1) = locations(1,1);    % phiの最低次ピークはそのまま採用
                judge = permute(phi_powers(i,2,locations(2,:)),[3,1,2])>permute(phi_powers(i,3,locations(2,:)),[3,1,2]); % ピーク周波数のパワーが相手方向より大きい部分の抽出
                winlocs = locations(2,judge);   % そのインデックスの取り出し
                obj.peak_locs(i,2) = winlocs(1);    % その中で最もインデックスの小さいものが，主モード周波数

                judge = permute(phi_powers(i,3,locations(3,:)),[3,1,2])>permute(phi_powers(i,2,locations(3,:)),[3,1,2]); % ピーク周波数のパワーが相手方向より大きい部分の抽出
                winlocs = locations(3,judge);   % そのインデックスの取り出し
                obj.peak_locs(i,3) = winlocs(1);    % その中で最もインデックスの小さいものが，主モード周波数
                %if (phi_powers(i,2,locations(2,1))>phi_powers(i,3,locations(2,1)))  %x方向最低ピークのパワーが，その周波数におけるy方向パワーよりも大きい
                %    obj.peak_locs(i,2) = locations(2,1);    % x方向最低ピークを，x方向主モードとして採用
                %else    
                for dim = 1:sp_dim
                    obj.x(i,dim,t) = 2*pi*f(obj.peak_locs(i,dim+1));   % 各方向第一ピーク周波数
                    obj.x(i,dim+sp_dim,t) = phi_powers(i,1,obj.peak_locs(i,dim+1))>phi_powers(i,dim+1,obj.peak_locs(i,dim+1));
                    obj.peak_power(i,2*dim-1,t) = phi_powers(i,1,obj.peak_locs(i,dim+1));
                    obj.peak_power(i,2*dim,t) = phi_powers(i,dim+1,obj.peak_locs(i,dim+1));
                end
            end
            %{
                for i = 1:obj.N
                    [pks,locs] = findpeaks(p(:,i),"MinPeakHeight",obj.power_threshold);       % ピーク検出
                    if isempty(locs)    % ピークはあったか？
                        obj.peak_locs(i,dim) = obj.N_band;    % 一番右のindexを格納
                    else
                        obj.peak_locs(i,dim) = locs(1);     % ピーク位置を格納
                        peak_freqs(i,dim) = f(locs(1)); % ピーク周波数を格納
                    end
                end
            end

            for i = 1:obj.N % 方向被りを解決
                if (phi_powers(i,2,obj.peak_locs(i,2))>phi_powers(i,3,obj.peak_locs(i,3))) % x方向が主モード
                    for loc = 2:length()
                else % y方向が主モード
                end
            end
            
            for dim = 1:sp_dim
                obj.x(:,dim,t) = f(obj.peak_locs(:,dim+1));   % 各方向第一ピーク周波数
                %obj.x(:,dim+sp_dim,t) = phi_powers(:,1,peak_locs(:,1))>phi_powers(:,dim+1,peak_locs(:,dim+1));   % 端っこ判定．1なら端っこ
                for i = 1:obj.N
                    %obj.x(i,dim+sp_dim,t) = phi_powers(i,1,peak_locs(i,1))>phi_powers(i,dim+1,peak_locs(i,dim+1));
                    obj.x(i,dim+sp_dim,t) = phi_powers(i,1,obj.peak_locs(i,dim+1))>phi_powers(i,dim+1,obj.peak_locs(i,dim+1));
                    obj.peak_power(i,2*dim-1,t) = phi_powers(i,1,obj.peak_locs(i,dim+1));
                    obj.peak_power(i,2*dim,t) = phi_powers(i,dim+1,obj.peak_locs(i,dim+1));
                end
            end
            %}
        end

        function obj = spatialDifferentiation(obj,t)
            % 空間偏微分の実施
            obj.input_sys.sys_robot = obj.input_sys.sys_robot.calcNablaMatrix(t);   % 偏微分行列の計算
            for dim = 1:obj.input_sys.sys_robot.spatial_dim
                Dinv = diag(1./sum( abs((ones(obj.N)-eye(obj.N)).*obj.input_sys.sys_robot.Nabla(:,:,dim)), 2) );
                % なぶら行列の対角以外の要素の，絶対値の和を対角成分に持つ行列．次数行列みたいな
                %D = diag(1./sum(abs(obj.input_sys.sys_robot.Nabla(:,:,dim)),2)); % 次数が並んだ縦ベクトル
                TF = isinf(Dinv);
                Dinv(TF) = 0;  % もし0割りポイントがあったら，1に置き換え
                obj.partial_phi(:,dim,t) = Dinv*obj.input_sys.sys_robot.Nabla(:,:,dim)*obj.input_sys.sys_cos.x(:,1,t);
                %正規化しない：obj.partial_phi(:,dim,t) = obj.input_sys.sys_robot.Nabla(:,:,dim)*obj.input_sys.sys_cos.x(:,1,t);
            end
        end

        function f_sp = spatialization(obj,f,x,t)
            % xを使ってfの空間方向差分を求める
            % f : 対象のスカラ関数 Na*1*Ntベクトル
            % x : エージェント座標 Na*sp_dim*Nt
            % t : 対象時刻
            Na = length(f(:,:,t));  % エージェント数
            obj.input_sys.sys_robot = obj.input_sys.sys_robot.calcGraphMatrices(t); % グラフ関連行列の計算
            Adj = obj.input_sys.sys_robot.Adj;  % 隣接行列
            Deg = obj.input_sys.sys_robot.Deg;  % 次数行列
            f_sp = zeros(size(x,1),size(x,2));  % 返り値のサイズはエージェント数×空間次元
            f_diff = Adj.*(repmat(f(:,1,t),1,Na)-repmat(f(:,1,t),1,Na).'); % fの差分行列(エッジある間のみ)
            x_diff(:,:,1) = repmat(x(:,1,t),1,Na)-repmat(x(:,1,t),1,Na).';  % 方向毎の差分行列
            x_diff(:,:,2) = repmat(x(:,2,t),1,Na)-repmat(x(:,2,t),1,Na).';  % 本当はsp_dimでforループ回すべき
            x_diff = normalize(x_diff,3,'norm'); % 空間方向に単位ベクトル化
            x_diff = fillmissing(x_diff,'constant',0);  % NaNが出たら0で埋める．（主に対角成分）
            f_sp_mat = zeros(Na,Na,2);
            f_sp_mat = pagemtimes(f_diff,x_diff); % 方向毎に計算
            f_sp = permute(pagemtimes(inv(Deg),sum(f_sp_mat,2)),[1,3,2]);
        end

        function obj = setParam(obj, t_with, threshold)
            %METHOD1 パラメータ設定
            %   詳細説明をここに記述
            obj.t_with = t_with;
            obj.power_threshold = threshold;
        end
    end
end

