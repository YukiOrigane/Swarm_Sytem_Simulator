function spectrumAnalyzePlot(phi,dt,t_range,predicts)
% 位相のスペクトラム解析を表示
%   詳細説明をここに記述
    t_vec = dt:dt:length(phi)*dt;
    if (ndims(phi)==3)
        phi = permute(phi(1,1,:),[1,3,2]);  % 整理して入っていない場合は，permute（番号1,次元1）で強制
    end
    [p1,f1] = pspectrum(phi(1,t_range),t_vec(t_range));
    loglog(2*pi*f1,p1,'LineWidth',1.5);
    grid on
    hold on
    for predict = predicts
        line([predict,predict],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
        %line([predict(2),predict(2)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
    end
    hold off
    xlabel("角周波数[rad/s]")
    ylabel("パワー")
    title("Spectrum of \Phi_1")
    ax = gca;
    ax.FontSize = 11;
    xlim([10^(-1),10^(2)])
end
