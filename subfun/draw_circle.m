function [ output_args ] = draw_circle (x,y,r,n)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
% if(nargin==4)
%     color='-k';
% end
color='ybgcrkr';
% if (n == 5)
%     color ='r';
% else
%     color ='-k';
% end
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color(1,n),'LineWidth',3);
end

