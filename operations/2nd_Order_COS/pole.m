
r = 0:0.01:2;
xi_1 = 1-2*r-2*1i*sqrt(r-r.^2);
xi_2 = 1-2*r+2*1i*sqrt(r-r.^2);
plot(r,abs(xi_1),r,abs(xi_2),'LineWidth' ,1.1);
legend("$\zeta_+ = |1-2r+2i\sqrt{r-r^2}|$","$\zeta_- = |1-2r-2i\sqrt{r-r^2}|$",'Interpreter','latex');
ax = gca;
ax.FontSize = 11;
grid on
xlim([0,2]);
xlabel("r")
ylabel("|\zeta|")
ax = gca;
ax.FontSize = 12;