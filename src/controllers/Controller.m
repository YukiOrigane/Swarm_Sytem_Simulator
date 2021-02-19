
% 純化されたコントローラは何ができるべきか

classdef Controller
    properties
        u
    end
    methods
        function obj = Controller
            % コンストラクタ．オーバーライド推奨
        end
        function u = calcInput(obj,t,sys)   % システムを引数にとる
            % 入力を決定する．オーバーライド推奨
            obj.u = u;
        end
    end
end