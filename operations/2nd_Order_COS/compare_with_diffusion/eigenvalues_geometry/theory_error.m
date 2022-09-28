% 誤差は(理論値-近似値)/理論値
% ４台直線状エージェント，kappa = 10の下．
error = [6.7,5.7,3.0,1.5,0.6,0.3;24,22,10.8,5.2,2.0,1.0];
dt_list = [0.05,0.02,0.01,0.005,0.002,0.001];
hz_list = 1./dt_list;
semilogx(hz_list,error,'LineWidth',1.5)
legend("第一固有モード","第二固有モード")
title("モード座標の時定数 理論値と近似値の誤差曲線")
xlabel("サンプリング周波数（更新周期の逆数）[Hz]")
ylabel("誤差百分率 %")
ax = gca;
ax.FontSize = 11;
grid on
