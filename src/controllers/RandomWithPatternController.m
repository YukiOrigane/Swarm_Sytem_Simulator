
classdef RandomWithPatternController < Controller
    properties
        Kv = 2.0; % ランダム移動の速度 Default : 1.0
    end
    
    methods
        function obj = setParam(obj, Kv)
            obj.Kv = Kv;
        end
        function obj = calcInput(obj,t,sys)
            theta = 2*pi*rand(sys.N,1);
            v = 0.2;% * (5-abs(sys.x(:,1,t)));
            obj.u = obj.Kv*[v.*cos(theta),v.*sin(theta) + 0.7*sin(pi*sys.x(:,1,t)/2.5)];
            limit_list = [-5 5; -5 5]; % TODO:外す
            obj = obj.wallBound(t,sys,limit_list);
            obj.u = reshape(obj.u,[],1);
        end
        
        % limit_list = [min_x1, max_x1; min_x2, max_x2;...; min_xn, max_xn]
        function obj = wallBound(obj,t,sys,limit_list)
            n_fixed = size(limit_list, 1);
            %obj.u = obj.u .* (-1*((sys.x(:,1:n_fixed,t)<repmat(limit_list(:,1).',sys.N,1))|(sys.x(:,1:n_fixed,t)>repmat(limit_list(:,2).',sys.N,1))));
            
        end
    end
    
end