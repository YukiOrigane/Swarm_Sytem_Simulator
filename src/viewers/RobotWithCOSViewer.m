
classdef RobotWithCOSViewer < Viewer
    properties
        robot_view
        cos_view
        sys_xi
        sys_phi
    end
    
    methods
        function obj = RobotWithCOSViewer(sys,field)
            obj@Viewer(sys,field);
            obj.robot_view = PlaneModeViewer(sys.sys_robot,field);
            obj.cos_view = CoupledOscillatorViewer(sys.sys_cos,field);
        end
        
        function obj = phasePositionPlot(obj,t,view_line,target)
            if exist('target','var')
                if target == "gap"
                    obj.robot_view.plotPosition(t,obj.sys.sys_cos.x(:,1,t)-obj.sys.sys_cos.x(1,1,t),view_line);
                elseif target == "average_gap"
                    obj.robot_view.plotPosition(t,obj.sys.sys_cos.x(:,1,t)-mean(obj.sys.sys_cos.x(:,1,t)),view_line);    
                else
                    obj.robot_view.plotPosition(t,obj.sys.sys_cos.x(:,1,t),view_line);
                end
            else
                obj.robot_view.plotPosition(t,obj.sys.sys_cos.x(:,1,t),view_line);
            end
        end
        
        function obj = phaseMeshPlot(obj,t)
            obj.robot_view.plotMesh(t,obj.sys.sys_cos.x(:,1,t)-obj.sys.sys_cos.x(1,1,t));
        end
        
        function obj = phaseImagePlot(obj,t)
            obj.robot_view.plotImage(t,obj.sys.sys_cos.x(:,1,t)-obj.sys.sys_cos.x(1,1,t));
        end
        
        function phaseGapPlot(obj,set,base_num) % 表示エージェント集合,基準エージェント番号
            obj.cos_view.phaseGapPlot(set,base_num);
        end
        
        function phaseMeanPlot(obj,set) % 表示エージェント集合
            obj.cos_view.phaseGapPlot(set);
        end
        
        function phaseLinearPlot(obj,t,set)
            obj.robot_view.plotLinear(t,obj.sys.sys_cos.x(set,1,t)-mean(obj.sys.sys_cos.x(set,1,t)));
        end
        
        function obj = spectrumAnalyze(obj,t,controller) % モード特性を解析
            N = obj.sys.N;
            obj.sys.sys_robot = obj.sys.sys_robot.calcEignExpansion(t); % 固有展開
            coeff_mat_mode = ones(N,1)*[1 controller.gamma 0];
            coeff_mat_mode(:,3) = diag(controller.Kappa*obj.sys.sys_robot.Lambda); % 伝達関数の分子係数行列
            for mu = 1:N
                xi(mu) = tf([1],coeff_mat_mode(mu,:));
            end
            phi = obj.sys.sys_robot.P * xi.';
            obj.sys_xi = xi.';
            obj.sys_phi = phi;
        end
    end
end