
classdef PlaneModeViewer < PlaneBasicViewer
    properties
        V
        D
        d
    end
    
    methods
        function obj = PlaneModeViewer(sys,field)
            obj@PlaneBasicViewer(sys,field);
            
        end
        
        function obj = analyzeGraphMode(obj,t)
            obj.sys = obj.sys.calcGraphMatrices(t); % グラフ関連行列を計算してもらう
            [obj.V,obj.D] = eig(obj.sys.Lap);
            obj.d = diag(obj.D);
        end
            
        function spacialModeView(obj,t,mode_list,layout,type)
            %obj = obj.analyzeGraphMode(t);
            cnt = 0;
            for mode = mode_list
                cnt = cnt+1;
                subplot(layout(1),layout(2),cnt);
                if type == "Position"
                    obj.plotPosition(t,obj.V(:,mode));
                elseif type == "Mesh"
                    obj.plotMesh(t,obj.V(:,mode));
                end
                %obj.plot3d(t,obj.V(:,mode));
                %obj.plotScatter3d(t,obj.V(:,mode),obj.V(:,mode));
                
                title(string(mode)+" th mode ; \lambda_{"+string(mode)+"} = " +string(obj.d(mode)));
            end
        end
        
        function spatialSpectrumView(obj)
            plot(2:length(obj.d), sqrt(obj.d(2:end)),'*');
            title("spectrum $\sqrt{\lambda_i}$",'Interpreter','latex');
            xlabel("i th mode");
            ylabel("spectrum")
            set(gca,'Fontsize',12);
            grid on
        end
    end
end