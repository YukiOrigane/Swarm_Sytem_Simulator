
classdef ChlamydomonasViewer < Viewer
    properties
        sys2
    end
    
    methods
        function obj = ChlamydomonasViewer(sys,sys2,field)
            obj@Viewer(sys,field);
            obj.sys2 = sys2;
        end
        
        function obj = ChlamydoPositionPlot(obj,t)
            %{
            [X,Y,Z] = sphere;   % 単位球面の生成
            r = 10;
            X2 = X * r;
            Y2 = Y * r;
            Z2 = Z * r;
            hold on
            s = surf(X2 + obj.sys.x(1,1,t), Y2 + obj.sys.x(1,2,t), Z2 + obj.sys.x(1,3,t));
            s.EdgeColor = 'none';
            s.FaceColor = '#77AC30';
            %}
            x = permute(obj.sys.x(:,:,1:t),[1,3,2]); % [エージェント数,時間,次元]
            plot3(x(1,:,1),x(1,:,2),x(1,:,3))
            grid on
            hold on
            u = obj.sys2.x(1,3,t);
            v = obj.sys2.x(1,4,t);
            w = obj.sys2.x(1,5,t);
            l = vecnorm([u,v,w]);
            sc = 20;
            s = scatter3(obj.sys.x(1,1,t), obj.sys.x(1,2,t), obj.sys.x(1,3,t),'filled');
            s.MarkerFaceColor = 'green';
            q = quiver3(obj.sys.x(1,1,t), obj.sys.x(1,2,t), obj.sys.x(1,3,t), u/l*sc,v/l*sc,w/l*sc);
            q.LineWidth = 2.0;
            q.Color = 'red';
            q = quiver3(obj.sys.x(1,1,t), obj.sys.x(1,2,t), obj.sys.x(1,3,t), obj.sys2.x(1,4,t)*sc,obj.sys2.x(1,5,t)*sc,obj.sys2.x(1,6,t)*sc);
            q.LineWidth = 1.0;
            q.Color = 'blue';
            ax = gca;
            xlim(obj.field.field_range(1,:));
            ylim(obj.field.field_range(2,:));
            zlim(obj.field.field_range(3,:));
            pbaspect([4 4 10])
        end
    end
end