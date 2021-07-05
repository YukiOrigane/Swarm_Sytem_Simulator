% アニメーションを生成する子
classdef Animation
    properties
        sys     % システム（複数システムのベクトルも許容）
        viewers % Viewerの配列
        F       % フレーム
        speed   % フレームの記録スピード（整数）
    end
    
    methods
        function obj = Animation(viewers)
            obj.viewers = viewers;
        end
        
        function obj = play(obj,func,speed)
            disp("Animation : アニメーション描画を開始します")
            figure
            obj.F = [];
            if isempty(speed)
                speed = 1;
            end
            obj.speed = speed;
            for t = 1:speed:obj.viewers(1).sys.Nt
                if strcmp(get(gcf,'currentcharacter'),'q')  % key stop
                    break;
                end

                func(obj.viewers,t);
                hold off
                drawnow;
                obj.F = [obj.F, getframe];%軸含めない
            end
            disp("Animation : アニメーション描画を終了します")
        end
        
        function obj = save(obj,name,save_speed)
            disp("Animation : 動画の保存を開始します")
            if isempty(save_speed)
                save_speed = 1;
            end
            fps = round((1/obj.viewers(1).sys.dt)/obj.speed/save_speed);
            if isempty(name)
                name = strcat('movie',datestr(datetime('now'), 30));
            end
            v = VideoWriter(name,'MPEG-4');
            v.FrameRate = fps;
            open(v);
            writeVideo(v,obj.F);
            close(v);
            disp("Animation : 動画が保存されました")
        end
        
    end % methods
    
end % Animation