% second_COS_impulseが既に走っていることを前提とする
% Analyzerみたいなクラスを作るかは要検討

%load(deformation_test.m)

Phi = permute(swarm.sys_cos.x(:,1,:),[1,3,2]);
%Phi = x;%-Omega_0*t_vec;

power_threshold = 0;%10^(-10);

t_with = 1000;  % FFtで使う観測カウントの数
obs_i = 5;  % 観測エージェント

analyze.fft_result = zeros(Nt,4096);   % エージェント数分増やすことも検討…
analyze.fft_freq = zeros(Nt,4096);
analyze.peak_locs = zeros(Nt,1);
analyze.peak_locs(1,1) = 1;
analyze.peak_freq = zeros(Nt,1);
analyze.options = @setFigureOptions;
analyze.sys.Nt = Nt;
analyze.sys.dt = dt;
analyze.degree_vec = zeros(Nt,1);

%% 計算
for t = 2:Nt
    if t<=t_with
        t_vec_part = t_vec(1:t);
        phi_part = Phi(obs_i,1:t);
    else
        t_vec_part = t_vec(t-t_with:t);
        phi_part = Phi(obs_i,t-t_with:t);
    end
    [p1,f1] = pspectrum(phi_part,t_vec_part);
    analyze.fft_result(t,:) = p1;
    analyze.fft_freq(t,:) = f1;
    [pks,locs] = findpeaks(p1);
    if isempty(locs)
        analyze.peak_locs(t,1) = 1;
    else
        for mu = 1:length(pks)  % 閾値処理
            if pks(mu) > power_threshold
                break
            end
        end
        %analyze.peak_locs(t,1) = locs(1);   % 最小局所最大値のインデックス
        analyze.peak_locs(t,1) = locs(mu);   % 最小局所最大値のインデックス
    end
    analyze.peak_freq(t,1) = 2*pi*f1(analyze.peak_locs(t,1));
end

%% 台数推定結果
figure
plot(t_vec(1:Nt), pi*sqrt(kappa)./analyze.peak_freq(:,1));
ax = gca;
ax.FontSize = 15;
xlabel("Time (s)")
ylabel("Estimated Length of Swarm")
title("Length Estimation")
grid on


%% 特定時刻
To = 62;
to = To/dt;
%predict = [5.24, 7.85];
figure
loglog(2*pi*analyze.fft_freq(to,:), analyze.fft_result(to,:), 'LineWidth',1.2);
hold on
scatter(2*pi*analyze.fft_freq(to,analyze.peak_locs(to,1)), analyze.fft_result(to,analyze.peak_locs(to,1)), 80, 'LineWidth', 2 );%, 15, '#D95319' );
%line([predict(1),predict(1)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
%line([predict(2),predict(2)],[10^(-25);10^0],'Color','#77AC30','LineStyle','--','LineWidth',0.1)
analyze.options(to);
title("");
legend("パワースペクトラム","推定値","理論値（N=6）","理論値（N=4）",'FontSize',12);
if (power_threshold>0)
    line([10^(0),10^(2)],[power_threshold,power_threshold],'Color','#77AC30','LineStyle','-','LineWidth',0.1)
end
legend("パワースペクトラム","推定値","理論値（N=6）","理論値（N=4）","パワー閾値",'FontSize',12);
ax = gca;
ax.FontSize = 15;

%% アニメーション
anime = Animation(analyze);

anime = anime.play(@snap,5);
%anime.save([],[]);

function snap(analyze,t)
    loglog(2*pi*analyze.fft_freq(t,:), analyze.fft_result(t,:), 'LineWidth',1.2);
    hold on
    scatter(2*pi*analyze.fft_freq(t,analyze.peak_locs(t,1)), analyze.fft_result(t,analyze.peak_locs(t,1)) , 80, 'LineWidth', 2);%, 15, '#D95319' );
    analyze.options(t);
end

function setFigureOptions(t)
    grid on
    hold on
    %predict = [5.24, 7.85];
    predict = [1.66, 2,48];
    line([predict(1),predict(1)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
    line([predict(2),predict(2)],[10^(-25);10^0],'Color','#77AC30','LineStyle','--','LineWidth',0.1)
    hold off
    xlabel("角周波数[rad/s]")
    ylabel("パワー")
    title("Spectrum of \Phi,t="+string(t))
    ax = gca;
    ax.FontSize = 11;
    xlim([10^(0),10^(2)])
    ylim([10^(-20),10^(0)])
    %legend("パワースペクトラム","推定値","理論値（N=6）","理論値（N=4）",'FontSize',12);
    text(30,10^-5,"t="+string(t*0.02)+" s",'FontSize',15);
    ax = gca;
    ax.FontSize = 15;
end