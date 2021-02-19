
% 要素間ユークリッド距離の計算
% 引数：x 実行列(エージェント数×次元)

function [Dist,Diff] = calEuclidDist(x)
    N = size(x,1);
    dim = size(x,2);
    Diff = zeros(N,N,dim);    % 差分行列:differencial matrices
    for i = 1:dim
        Diff(:,:,i) = repmat(x(:,i),1,N)-repmat(x(:,i).',N,1); 
    end
    Dist = vecnorm(Diff,2,3);   % 距離行列：差分行列の，次元方向に2ノルムを計算
end