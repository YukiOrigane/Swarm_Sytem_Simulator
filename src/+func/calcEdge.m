% �����s������ɁC�d�ݖ����O���t��Ԃ�
function [Lap,Adj] = calcEdge(Dist,rv)
    Adj = Dist < rv;  % �אڍs��F�������ϑ������ȉ����ۂ��i�d�ݖ����j
    Deg = sum(Adj,1);   % �����s��
    Lap = Deg-Adj;        % �O���t���v���V�A��
end

