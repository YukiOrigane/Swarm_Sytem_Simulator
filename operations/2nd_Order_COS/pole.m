
r = -2:0.1:2;
xi_1 = 1-2*r-2*1i*sqrt(r-r.^2);
xi_2 = 1-2*r+2*1i*sqrt(r-r.^2);
plot(r,abs(xi_1),r,abs(xi_2));
legend("$|1-2r+2i\sqrt{r-r^2}|$","$|1-2r-2i\sqrt{r-r^2}|$",'Interpreter','latex');
ax = gca;
ax.FontSize = 11;
grid on
xlabel("r")
ylabel("|\xi_i|")