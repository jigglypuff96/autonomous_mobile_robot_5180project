function [X,P,score] = SPF(X,P,Wm,Wc,u,y,sigma_f,nx,nw,nv,f,h)
%  INPUT
%       X:      n-by-3 state variable n = nx+nw+nv
%       P:      n-by-n state variable P = [[Pxx, 0, 0]]
%                                          [0,   Q, 0]
%                                          [0,   0, R]]
%       Wm:     1-by-2*na+1 weights
%       Wc:     1-by-weights
%       u:      whatever-by-1 input
%       y:      m-by-1 measurement
%       sigma_f:sigma_f value for SPF
%       f:      dynamic model function f(x,u) (+process noise)
%       h:      measurement model function h(x) (+measuremnet noise)
%
%   OUTPUT
%       X:      n-by-1 state variable n = nx+nw+nv
%       P:      n-by-n state variable P = [[Pxx, 0, 0]]
%                                          [0,   Q, 0]
%                                          [0,   0, R]]

% Seperate vairable
na = nw+nv+nx;
% xhat = X(1:nx,:);
% xhata = [xhat;zeros(nw,1);zeros(nv,1)];
% X = [xhata, xhata*ones(1,na)-sigma_f*P^0.5,xhata*ones(1,na)+sigma_f*P^0.5 ];
score = 0;


Xxi = X(1:nx,:);
Xwi = X((nx+1):(nw+nx),:);
Xvi = X(((nw+nx)+1):end,:);  %Xv_ = Xv_i
Pxx = P(1:nx,1:nx);
Q =  P((nx+1):(nx+nw),(nx+1):(nx+nw));
R = P((nx+nw+1):end,(nx+nw+1):end);
%Predict
Xxi_ = zeros(nx,na*2+1);
for i = 1:2*na+1
    Xxi_(:,i) = f(Xxi(:,i),u,Xwi(:,i));
end
xhat_ = zeros(nx,1);
for i = 1:2*na+1
    xhat_ = xhat_+Wm(i)*Xxi_(:,i);
end
Pxx_ = zeros(nx,nx);
for i = 1:2*na+1
    Pxx_ = Pxx_+(Wc(i)*(Xxi_(:,i)-xhat_)*(Xxi_(:,i)-xhat_)');
end

%update
m = length(y);
yi_ = zeros(m,na*2+1);
for i = 1:2*na+1
    yi_(:,i) = h(Xxi(:,i),Xvi(:,i));
end


index = find(y==0);
y(index) = [];
yi_(index,:) = [];
m = length(y);




yhat_ = zeros(m,1);
for i = 1:2*na+1
    yhat_ = yhat_+Wm(i)*yi_(:,i);
end
Pyy_ = zeros(m,m);

for i = 1:2*na+1
    Pyy_ = Pyy_+(Wc(i)*(yi_(:,i)-yhat_)*(yi_(:,i)-yhat_)');
end




%================= check ===============================
for i = 1:2*na+1
    score = score+sum(Wc(i)*(y'-yhat_).^2);
end


% ================ check ===============================



Pxy_ = zeros(nx,m);
for i = 1:2*na+1
    Pxy_ = Pxy_+(Wc(i)*(Xxi_(:,i)-xhat_)*(yi_(:,i)-yhat_)');
end
K = Pxy_*(inv(Pyy_));
xhat = xhat_+K*(y'-yhat_);
Pxx = Pxx_ - K*Pyy_*K';

xhata = [xhat;zeros(nw,1);zeros(nv,1)];
P = [[Pxx, zeros(nx,nw), zeros(nx,nv)];[zeros(nw,nx),   Q, zeros(nw,nv)];[zeros(nv,nx),   zeros(nv,nw), R]];
%score = trace(Pyy_.^2);
P(P<0) = 0;

X = [xhata, xhata*ones(1,na)-sigma_f*P^0.5,xhata*ones(1,na)+sigma_f*P^0.5 ];
