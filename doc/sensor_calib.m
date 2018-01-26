function sensor_calib
global R_nv R_mi

R_nv=expm(hat(rand(3,1)));
R_mi=expm(hat(rand(3,1)));

R_nv=expm_SO3([0.1 0.2 0.3]);
R_mi=expm_SO3([pi 0 0]);

N=10;
t=linspace(0,9,N);


for k=1:N
    [R_ni(:,:,k) R_vm(:,:,k)]=measurement(t(k));
end

ZX=zeros(3,3);
ZY=zeros(3,3);
for i=1:N
    for j=1:N
        r_ij=logm_SO3(R_ni(:,:,i)*R_ni(:,:,j)');
        q_ij=logm_SO3(R_vm(:,:,i)*R_vm(:,:,j)');
        ZX=ZX+r_ij*q_ij';
        r_ij=logm_SO3(R_ni(:,:,j)'*R_ni(:,:,i));
        q_ij=logm_SO3(R_vm(:,:,j)'*R_vm(:,:,i));
        ZY=ZY+r_ij*q_ij';
    end
end

[U S V]=svd(ZX);
R_nv_new=U*diag([1 1 det(U)*det(V)])*V';

[U S V]=svd(ZY);
R_mi_new=V*diag([1 1 det(U)*det(V)])*U';


disp([R_nv R_nv_new]);
norm(R_nv-R_nv_new)

disp([R_mi R_mi_new]);
norm(R_mi-R_mi_new)        

filename='sensor_calib_0';
save(filename);
evalin('base',['load ' filename]);

end

function [R_ni R_vm]=measurement(t)
global R_nv R_mi

R_vm = [cos(t), -cos(t)*sin(t), sin(t)^2;
    cos(t)*sin(t), cos(t)^3 - sin(t)^2, -cos(t)*sin(t) - cos(t)^2*sin(t);
    sin(t)^2, cos(t)*sin(t) + cos(t)^2*sin(t), cos(t)^2 - cos(t)*sin(t)^2];

R_ni=R_nv*R_vm*R_mi;


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









