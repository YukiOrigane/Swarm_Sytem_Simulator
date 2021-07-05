
classdef GeneralPlotViewer < Viewer
    methods
        % システムの時間推移を単純に描画 d:状態次元, N_range:表示するエージェント番号のリスト
        function timeTransitionView(obj,d,N_range)
            plot(obj.sys.t_vec, permute(obj.sys.x(N_range,d,:),[1,3,2]))
            lgd = legend(string(N_range));
            lgd.NumColumns = round(length(N_range)/5);
            hold on
        end
    end
end