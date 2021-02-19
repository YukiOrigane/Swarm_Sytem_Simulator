% 二次元平面上を扱うときの基本的な描画
% 状態の第一量がx（horizon），第二量がy (vertical)であることを仮定
classdef PlaneBasicViewer < Viewer
    methods
        function obj = PlaneBasicViewer(sys,field)
            obj@Viewer(sys,field);
        end
        
        function plotPosition(obj,t,col)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            if ~exist('col','var')
                col = zeros(obj.sys.N,1);   % 設定されてなければ同じ色
                colorbar
                hold on
            end
            obj.sys.calcGraphMatrices(t);
            triuAdj = triu(obj.sys.Adj,1);      % 隣接行列の上三角成分取り出し
            for i = 1:obj.sys.N
                for j = 1:obj.sys.N
                    if triuAdj(i,j) ~= 0    % 隣接してるか？
                        plot([x(i),x(j)],[y(i),y(j)],'k-','LineWidth',2) % エージェント間の線，for文消せるんか？
                        hold on
                    end
                end
            end
            scatter(x(:),y(:),150,col,'filled','MarkerEdgeColor','k','LineWidth',2);
            colormap cool
            set(gca,'FontSize',12);
            axis([obj.field.getFieldRangeX(),obj.field.getFieldRangeY()])
            pbaspect([obj.field.getFieldSize().' 1])%アスペクト比
            grid on
        end
    end
end