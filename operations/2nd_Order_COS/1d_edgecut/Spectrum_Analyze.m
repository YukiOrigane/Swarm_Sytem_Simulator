% second_COS_impulseが既に走っていることを前提とする
% Analyzerみたいなクラスを作るかは要検討

x = permute(swarm.sys_cos.x(:,1,:),[1,3,2]);
Phi = x;%-Omega_0*t_vec;
%%%%%%%% 予測
i = 1;


%%%%%%%% Phiの方
%% 震源地
t_range_before = 1000:2000;
t_range_mid = 2500:3500;
t_range_after = 4000:5000;
%t_range_after = 1:Nt;
n_set = [1,2,3,4];
lw = 1.0;

figure
%predict = [1.99, 10^-5];
subplot(1,3,1)
for i = n_set
    [p1,f1] = pspectrum(Phi(i,t_range_before),t_vec(t_range_before));
    loglog(2*pi*f1,p1,'LineWidth',lw)
    grid on
    hold on
end
%line([predict(1),predict(1)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
hold off
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("切断前（t \in [ "+string(t_range_before(1)*dt)+", "+string(t_range_before(end)*dt)+"]）")
legend(string(n_set));
ax = gca;
ax.FontSize = 11;
xlim([10^(-1),10^(2)])

subplot(1,3,2)
for i = n_set
    [p1,f1] = pspectrum(Phi(i,t_range_mid),t_vec(t_range_mid));
    loglog(2*pi*f1,p1,'LineWidth',lw)
    grid on
    hold on
end
%line([predict(1),predict(1)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
hold off
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("切断中（t \in [ "+string(t_range_mid(1)*dt)+", "+string(t_range_mid(end)*dt)+"]）")
legend(string(n_set));
ax = gca;
ax.FontSize = 11;
xlim([10^(-1),10^(2)])

subplot(1,3,3)
for i = n_set
    [p1,f1] = pspectrum(Phi(i,t_range_after),t_vec(t_range_after));
    loglog(2*pi*f1,p1,'LineWidth',lw)
    grid on
    hold on
end
%line([predict(1),predict(1)],[10^(-25);10^0],'Color','#D95319','LineStyle','--','LineWidth',0.1)
hold off
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("再結合後（t \in [ "+string(t_range_after(1)*dt)+", "+string(t_range_after(end)*dt)+"]）")
legend(string(n_set));
ax = gca;
ax.FontSize = 11;
xlim([10^(-1),10^(2)])