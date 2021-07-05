
classdef VirtualForceController < Controller
    properties
        % see 折金のSymmetry 2021
        k_P     % 位相勾配力係数
        k_F     % 群形成力係数
        k_O     % 衝突回避力係数
        d_O     % 衝突回避距離
        r_C     % 平衡距離
    end
    
    methods
        function obj = calcInput(obj, t, sys)
        end
    end
    
end