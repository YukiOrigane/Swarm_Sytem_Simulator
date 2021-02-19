% 距離行列を元に，重み無しグラフを返す
function [Lap,Adj] = calcEdge(Dist,rv)
    Adj = Dist < rv;  % 隣接行列：距離が観測距離以下か否か（重み無し）
    Deg = sum(Adj,1);   % 次数行列
    Lap = Deg-Adj;        % グラフラプラシアン
end

