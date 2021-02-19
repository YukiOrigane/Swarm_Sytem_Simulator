% 状態から線形FBを行う
classdef LinearFeedBackController < Controller
    properties
        Gain    % ゲイン行列
        ref     % 目標値
        len     % 入力ベクトル長さ
    end
    methods
        function obj = setGainMatrix(obj,Gain)  % ゲイン行列の指定
            obj.Gain = Gain;
        end
        function obj = setDiagGainMatrix(obj,gain) % 対角成分のみを指定(Na,dim)行列
            Na = size(gain,1); dim = size(gain,2);
            obj = obj.setGainMatrix( diag(reshape(gain,Na*dim,1)) );
        end
        function obj = setReference(obj,ref)    % 目標値の指定
            obj.ref = ref;
        end
        function obj = calcInput(obj,t,sys,ref)       % 制御入力の計算
            obj = obj.setReference(ref);
            sysNa = sys.N; sysdim = sys.dim;
            obj.u = obj.Gain*(reshape(obj.ref-sys.x(:,:,t), sysNa*sysdim, 1));
        end
    end
end