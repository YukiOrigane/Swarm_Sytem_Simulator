
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
            arguments
                obj
                t
                mode_list
                layout
                type {mustBeMember(type,["Position","Mesh","Linear"])} = "Position";
            end
            %obj = obj.analyzeGraphMode(t);
            obj = sortMode(obj);
            cnt = 0;
            for mode = mode_list
                cnt = cnt+1;
                if (layout(1)>1 || layout(2)>1)
                    subplot(layout(1),layout(2),cnt);
                end
                if type == "Position"
                    obj.plotPosition(t,obj.V(:,mode));
                elseif type == "Mesh"
                    obj.plotMesh(t,obj.V(:,mode));
                elseif type == "Linear"
                    obj.plotLinear(t,obj.V(:,mode));
                    grid on
                end
                %obj.plot3d(t,obj.V(:,mode));
                %obj.plotScatter3d(t,obj.V(:,mode),obj.V(:,mode));
                
                title(string(mode-1)+" th mode ; $$\sqrt{\sigma}_{"+string(mode-1)+"}$$ = " +string(sqrt(obj.d(mode)))+ " rad/m", 'Interpreter','latex');
            end
        end
        
        function spatialSpectrumView(obj)
            plot(1:length(obj.d), sqrt(abs(obj.d(1:end))),'*');
            title("spectrum $\sqrt{\lambda_i}$",'Interpreter','latex');
            xlabel("i th mode");
            ylabel("spectrum")
            set(gca,'Fontsize',12);
            grid on
        end

        function obj = sortMode(obj)    % モードを小さい順に並び変え
            [obj.d, order] = sort(obj.d);
            obj.V = obj.V(:,order);
            obj.D = obj.D(:,order);
        end
    end
end