function pos_att_est
global R_nv R_mi R_fv R_bi;
global Q_W Q_a V_I_R V_V_R V_V_x;
global Q_b_a Q_b_W;
global g e3;
global b_a b_W;
close all;

g=9.81;
e3=[0 0 1]';

R_fv=[0 1 0; 1 0 0; 0 0 -1];
R_mi=expm_SO3(rand(3,1));
R_nv=expm_SO3(rand(3,1));
R_bi=expm_SO3(rand(3,1));
Q_W=diag([0.5 0.5 0.5]).^2;
Q_a=diag([0.1 0.1 0.1]).^2;
Q_b_a=diag([0.0001 0.0001 0.0001]).^2;
Q_b_W=Q_W;%diag([0.0001 0.0001 0.0001]).^2;

V_I_R=diag([0.01 0.01 0.01]).^2;
V_V_R=diag([0.1 0.1 0.1]).^2;
V_V_x=diag([0.1 0.1 0.1]).^2;
Q=zeros(12,12);
Q(1:3,1:3)=Q_a;
Q(4:6,4:6)=Q_W;
Q(7:9,7:9)=Q_b_a;
Q(10:12,10:12)=Q_b_W;

T=[zeros(3) zeros(3) eye(3) zeros(3) zeros(3);
    zeros(3) zeros(3) zeros(3) zeros(3) eye(3);
    eye(3) zeros(3) zeros(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3) zeros(3) zeros(3);
    zeros(3) zeros(3) zeros(3) eye(3) zeros(3)];
T_o=[eye(3) zeros(3) zeros(3) zeros(3) zeros(3);
    zeros(3) eye(3) zeros(3) zeros(3) zeros(3)];


tf=100;
N=10001;
t=linspace(0,tf,N);
h=t(2)-t(1);

b_a=0.1*[0.5 0.4 0.3]';
b_W=0.1*[0.1 0.2 0.05]';
for k=1:N
    [x_true(:,k), v_true(:,k), a_true(:,k) R_true(:,:,k) W_true(:,k)]=true(t(k));
end

% %IMU measurement check
% Q_W=zeros(3,3);Q_a=zeros(3,3);V_I_R=zeros(3,3);
% for k=1:N
%     [a_IMU(:,k) R_IMU(:,:,k) W_IMU(:,k)]=measuremenet_IMU(t(k));
%     R(:,:,k)=R_fv*R_nv'*R_IMU(:,:,k)*R_bi';
%     W(:,k)=R_bi*(W_IMU(:,k)+b_W);
%     a(:,k)=R_true(:,:,k)*R_bi*(a_IMU(:,k)+b_a)+g*e3;
%     errR(k)=norm(R(:,:,k)-R_true(:,:,k));
%     errW(k)=norm(W(:,k)-W_true(:,k));
%     erra(k)=norm(a(:,k)-a_true(:,k));
% end
% plot(t,errR,'r',t,errW,'g',t,erra,'b');
% return;

% 
% %VICON measurement check
% V_V_x=zeros(3,3);V_V_R=zeros(3,3);
% for k=1:N
%     [x_VICON(:,k) R_VICON(:,:,k)]=measuremenet_VICON(t(k));
%     R(:,:,k)=R_fv*R_VICON(:,:,k)*R_mi*R_bi';
%     x(:,k)=R_fv*x_VICON(:,k);
%     errR(k)=norm(R(:,:,k)-R_true(:,:,k));
%     errx(k)=norm(x(:,k)-x_true(:,k));
% end
% plot(t,errR,t,errx);

for k=1:N
    [a_IMU(:,k) R_IMU(:,:,k) W_IMU(:,k)]=measuremenet_IMU(t(k));
    [x_VICON(:,k) R_VICON(:,:,k)]=measuremenet_VICON(t(k));
end

X=zeros(15,N);
x_bar=zeros(3,N);
v_bar=zeros(3,N);
a_bar=zeros(3,N);
R_bar=zeros(3,3,N);
b_a_bar=zeros(3,N);
b_W_bar=zeros(3,N);

P=zeros(15,15,N);
x_bar(:,1)=[0 0 0]';%x_true(:,1);%
v_bar(:,1)=[0 0 0]';%v_true(:,1);
R_bar(:,:,1)=eye(3);%R_true(:,:,1);%eye(3);
b_a_bar(:,1)=[0 0 0]';
b_W_bar(:,1)=[0 0 0]';

a_bar(:,1)=R_bar(:,:,1)*R_bi*(a_IMU(:,1)+b_a_bar(:,1))+g*e3;
W_bar(:,1)=R_bi*(W_IMU(:,1)+b_W_bar(:,1));

%              x            v            eta                b_a          b_W
P(:,:,1)=diag([1^2 1^2 1^2, 1^2 1^2 1^2, 0.1^2 0.1^2 0.1^2, 1^2 1^2 1^2, 0.1^2 0.1^2 0.1^2]);

for k=2:N
    
    % prediction
    b_a_bar(:,k)=b_a_bar(:,k-1);
    b_W_bar(:,k)=b_W_bar(:,k-1);

    W_bar(:,k)=R_bi*(W_IMU(:,k)+b_W_bar(:,k));
    R_bar(:,:,k)=R_bar(:,:,k-1)*expm_SO3(h/2*(W_bar(:,k-1)+W_bar(:,k)));

    a_bar(:,k)=R_bar(:,:,k)*R_bi*(a_IMU(:,k)+b_a_bar(:,k))+g*e3;
    
    x_bar(:,k)=x_bar(:,k-1)+h*v_bar(:,k-1)+h^2/2*a_bar(:,k-1);
    v_bar(:,k)=v_bar(:,k-1)+h/2*(a_bar(:,k-1)+a_bar(:,k));
    
    A=[zeros(3) eye(3) zeros(3) zeros(3) zeros(3);
        zeros(3) zeros(3) -R_bar(:,:,k-1)*hat(R_bi*(a_IMU(:,k-1)+b_a_bar(:,k-1))) R_bar(:,:,k-1)*R_bi zeros(3);
        zeros(3) zeros(3) -hat(R_bi*(W_IMU(:,k-1)+b_W_bar(:,k-1))) zeros(3) R_bi;
        zeros(3) zeros(3) zeros(3) zeros(3) zeros(3);
        zeros(3) zeros(3) zeros(3) zeros(3) zeros(3)];
    F=[zeros(3) zeros(3) zeros(3) zeros(3);
       R_bar(:,:,k-1)*R_bi zeros(3) zeros(3) zeros(3);
       zeros(3) R_bi zeros(3) zeros(3);
       zeros(3) zeros(3) eye(3) zeros(3);
       zeros(3) zeros(3) zeros(3) eye(3)];
    
    Psi=eye(15)+h/2*A;
    A_km=eye(15)+h*A*Psi;
    F_km=h*Psi*F;
    
    P(:,:,k)=A_km*P(:,:,k-1)*A_km'+F_km*Q*F_km';

    % R correction by IMU
    tmp=R_bar(:,:,k)'*R_fv*R_nv'*R_IMU(:,:,k)*R_bi';
    delz=1/2*vee(tmp-tmp');
    H=[zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];
    S=H*P(:,:,k)*H'+R_bi*V_I_R*R_bi';    
    K=P(:,:,k)*H'*inv(S);    
    
    P_o=T_o*T*P(:,:,k)*T'*T_o';
    H_o=H*T'*T_o';
    K_o=P_o*H_o'*inv(S);

    delchi=K_o*delz;
    R_bar(:,:,k)=R_bar(:,:,k)*expm_SO3(delchi(1:3));
    b_W_bar(:,k)=b_W_bar(:,k)+delchi(4:6);

    K=T'*T_o'*K_o;
    P(:,:,k)=(eye(15)-K*H)*P(:,:,k)*(eye(15)-K*H)'+K*R_bi*V_I_R*R_bi'*K';
   
%    keyboard;
%    P_o=(eye(6)-K_o*H_o)*P_o*(eye(6)-K_o*H_o)'+K_o*R_bi*V_I_R*R_bi'*K_o';
%    P(:,:,k)=P(:,:,k)+T'*T_o'*(P_o-T_o*T*P(:,:,k)*T'*T_o')*T_o*T;
    disp(min(eig(P(:,:,k))));
    
    % VICON correction
    tmp=R_bar(:,:,k)'*R_fv*R_VICON(:,:,k)*R_mi*R_bi';
    delz=[1/2*vee(tmp-tmp'); x_VICON(:,k)-R_fv'*x_bar(:,k)];
    H=[zeros(3) zeros(3) eye(3) zeros(3) zeros(3);
        R_fv' zeros(3) zeros(3) zeros(3) zeros(3)];
    G=[R_bi*R_mi' zeros(3);
        zeros(3) eye(3)];
    V=[V_V_R zeros(3);
        zeros(3) V_V_x];
    S=H*P(:,:,k)*H'+G*V*G';
    
    K=P(:,:,k)*H'*inv(S);
    delx=K*delz;
    x_bar(:,k)=x_bar(:,k)+delx(1:3);
    v_bar(:,k)=v_bar(:,k)+delx(4:6);
    R_bar(:,:,k)=R_bar(:,:,k)*expm_SO3(delx(7:9));
    b_a_bar(:,k)=b_a_bar(:,k)+delx(10:12);
    b_W_bar(:,k)=b_W_bar(:,k)+delx(13:15);

    P(:,:,k)=(eye(15)-K*H)*P(:,:,k)*(eye(15)-K*H)'+K*G*V*G'*K';
    
    
end

for k=1:N
    eta(:,k)=logm_SO3(R_bar(:,:,k)'*R_true(:,:,k));
    sigma(:,k)=sqrt(diag(P(:,:,k)));
end

figure;
plot(t,x_true,'r',t,x_bar,'b:');ylabel('$x$','interpreter','latex');
figure;
plot(t,v_true,'r',t,v_bar,'b:');ylabel('$v$','interpreter','latex');
figure;
plot(t,eta,'b:');ylabel('$\eta$','interpreter','latex');
figure;
plot(t,b_a*ones(1,N),'r',t,b_a_bar,'b:');ylabel('$\bar b_a$','interpreter','latex');
figure;
plot(t,b_W*ones(1,N),'r',t,b_W_bar,'b:');ylabel('$\bar b_\Omega$','interpreter','latex');
figure;
plot(t,W_true,'r',t,W_bar,'b:');ylabel('$\Omega$','interpreter','latex');
figure;
plot(t,a_true,'r',t,a_bar,'b:');ylabel('$a$','interpreter','latex');

figure;
for ii=1:3
    subplot(3,1,ii);
    plot(t,x_bar(ii,:)-x_true(ii,:),'b');
    hold on;
    plot(t,sigma(ii,:),'r:',t,-sigma(ii,:),'r:');
end
subplot(3,1,2);ylabel('$e_x$','interpreter','latex');
figure;
for ii=1:3
    subplot(3,1,ii);
    plot(t,v_bar(ii,:)-v_true(ii,:),'b');
    hold on;
    plot(t,sigma(ii+3,:),'r:',t,-sigma(ii+3,:),'r:');
end
subplot(3,1,2);ylabel('$e_v$','interpreter','latex');
figure;
for ii=1:3
    subplot(3,1,ii);
    plot(t,eta(ii,:),'b');
    hold on;
    plot(t,sigma(ii+6,:),'r:',t,-sigma(ii+6,:),'r:');
end
subplot(3,1,2);ylabel('$\eta$','interpreter','latex');




filename='pos_att_est_0';
save(filename);
evalin('base',['load ' filename]);

end

function [x v a R W]=true(t)

x=[1.2*sin(0.2*pi*t), 4.2*cos(0.1*pi*t), -0.5]';
v=[1.2*0.2*pi*cos(0.2*pi*t), -4.2*0.1*pi*sin(0.1*pi*t), 0]';
a=[-1.2*(0.2*pi)^2*sin(0.2*pi*t), -4.2*(0.1*pi)^2*cos(0.1*pi*t), 0]';
R=[cos(t), -cos(t)*sin(t), sin(t)^2;
    cos(t)*sin(t), cos(t)^3 - sin(t)^2, -cos(t)*sin(t) - cos(t)^2*sin(t);
    sin(t)^2, cos(t)*sin(t) + cos(t)^2*sin(t), cos(t)^2 - cos(t)*sin(t)^2];
W=[            cos(t) + 1
   sin(t) - sin(2*t)/2
 cos(t) - cos(t)^2 + 1];

end

function [a_i R_ni W_i]=measuremenet_IMU(t)
global R_nv R_mi R_fv R_bi;
global Q_W Q_a V_I_R V_V_R V_V_x;
global Q_b_a Q_b_W;
global g e3;
global b_a b_W;



[x v a_f R_fb W_b]=true(t);

w_a=mvnrnd(zeros(3,1),Q_a)';
a_i=R_bi'*R_fb'*a_f-R_bi'*R_fb'*(g)*e3+w_a-b_a;

w_W=mvnrnd(zeros(3,1),Q_W)';
W_i=R_bi'*W_b+w_W-b_W;

zeta=mvnrnd(zeros(3,1),V_I_R)';
R_ni=R_nv*R_fv'*R_fb*R_bi*expm_SO3(zeta);

end


function [x_v R_vm]=measuremenet_VICON(t)
global R_nv R_mi R_fv R_bi;
global Q_W Q_a V_I_R V_V_R V_V_x;

[x_f v a R_fb W]=true(t);

zeta_x=mvnrnd(zeros(3,1),V_V_x)';
x_v=R_fv'*x_f+zeta_x;

zeta_R=mvnrnd(zeros(3,1),V_V_R)';
R_vm=R_fv'*R_fb*R_bi*R_mi'*expm_SO3(zeta_R);
end


function r=logm_SO3(R)

[V lam]=eig(R);
eps=1e-6;
min_del_lam_1=1;
for i=1:3
    if norm(imag(V(:,i))) < eps
        if (lam(i,i)^2-1) < min_del_lam_1
            min_del_lam_1=lam(i,i)^2-1;
            i_min=i;
        end
    end
end
v=real(V(:,i_min));

cos_theta=(trace(R)-1)/2;
if cos_theta > 1.0
    cos_theta=1;
elseif cos_theta < -1
    cos_theta=-1;
end
theta=real(acos(cos_theta));
R_new=expm_SO3(theta*v);

if norm(R-R_new) > norm(R-R_new')
    v=-v;
end

r=v*theta;
end

function R=expm_SO3(r)
theta=norm(r);

y=sinx_over_x(theta);
y2=sinx_over_x(theta/2);

R=eye(3)+y*hat(r)+1/2*y2^2*hat(r)^2;
end

function y=sinx_over_x(x)
eps=1e-6;
if abs(x) < eps
    y=- x^10/39916800 + x^8/362880 - x^6/5040 + x^4/120 - x^2/6 + 1;
else
    y=sin(x)/x;
end
end




