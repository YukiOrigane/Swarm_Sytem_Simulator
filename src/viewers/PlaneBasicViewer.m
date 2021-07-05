% 二次元平面上を扱うときの基本的な描画
% 状態の第一量がx（horizon），第二量がy (vertical)であることを仮定
classdef PlaneBasicViewer < Viewer
    properties
        dx
        dy
        lx
        ly
        xq
        yq
    end
    
    methods
        function obj = PlaneBasicViewer(sys,field)
            obj@Viewer(sys,field);
        end
        
        function obj = setPlaneGrid(obj,dx,dy) % 空間分割ベクトルとグリッドを作る
            obj.lx = obj.field.getFieldRangeX(1):dx:obj.field.getFieldRangeX(2);
            obj.ly = obj.field.getFieldRangeY(1):dy:obj.field.getFieldRangeY(2);
            [obj.xq,obj.yq] = meshgrid( obj.lx, obj.ly);
        end
        
        function plotPosition(obj,t,col,view_line)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            if ~exist('col','var')
                    col = zeros(obj.sys.N,1);   % 設定されてなければ同じ色
            elseif isempty(col)
                col = zeros(obj.sys.N,1);   % 設定されてなければ同じ色
            end
                
            if ~exist('view_line','var')
                view_line = true;
            end
            obj.sys.calcGraphMatrices(t);
            if(view_line)
                triuAdj = triu(obj.sys.Adj,1);      % 隣接行列の上三角成分取り出し
                for i = 1:obj.sys.N
                    for j = 1:obj.sys.N
                        if triuAdj(i,j) ~= 0    % 隣接してるか？
                            plot([x(i),x(j)],[y(i),y(j)],'k-','LineWidth',1) % エージェント間の線，for文消せるんか？
                            hold on
                        end
                    end
                end
            end
            scatter(x(:),y(:),150,col,'filled','MarkerEdgeColor','k','LineWidth',1.5);
            colormap cool
            set(gca,'FontSize',12);
            axis([obj.field.getFieldRangeX(),obj.field.getFieldRangeY()])
            pbaspect([obj.field.getFieldSize().' 1])%アスペクト比
            grid on
        end
        
        function plotPositionNumber(obj,t,col,view_line)
            obj.plotPosition(t,col,view_line);
            text(obj.sys.x(:,1,t),obj.sys.x(:,2,t),string(1:obj.sys.N),'FontSize',11,'HorizontalAlignment', 'center','Color','white');
        end
        
        function plot3d(obj,t,z)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            plot3(x,y,z,'o');
        end
        
        function plotScatter3d(obj,t,z,col)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            if ~exist('col','var')
                col = zeros(obj.sys.N,1);   % 設定されてなければ同じ色
                %colorbar
                hold on
            end
            scatter3(x,y,z,30,col,'filled');
        end
        
        function plotLinear(obj,t,v)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            plot(x,v);
            hold on
            plot(x,v,'*');
            set(gca,'FontSize',12);
        end
        
        function plotMesh(obj,t,v)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            dx = 0.1; dy = 0.1;
            [xq,yq] = meshgrid( obj.field.getFieldRangeX(1):dx:obj.field.getFieldRangeX(2), obj.field.getFieldRangeY(1):dy:obj.field.getFieldRangeY(2));
            vq = griddata(x,y,v,xq,yq,'cubic');
            mesh(xq,yq,vq);
            set(gca,'FontSize',12);
        end
        
        function plotImage(obj,t,v)
            x = obj.sys.x(:,1,t);
            y = obj.sys.x(:,2,t);
            dx = 0.1; dy = 0.1;
            [xq,yq] = meshgrid( obj.field.getFieldRangeX(1):dx:obj.field.getFieldRangeX(2), obj.field.getFieldRangeY(1):dy:obj.field.getFieldRangeY(2));
            vq = griddata(x,y,v,xq,yq,'cubic');
            imagesc(x,y,vq);
            set(gca,'FontSize',12);
        end
        
    end
end