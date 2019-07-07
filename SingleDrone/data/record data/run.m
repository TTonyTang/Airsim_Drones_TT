vs=importdata("vs.txt");
rs=importdata("rs.txt");
num=length(vs);
rs=[0 0;rs];

rx=rs(:,1);
ry=rs(:,2);

vx=vs(:,1);
vy=vs(:,2);

num=length(vs);

rr=zeros(num,1);
vv=zeros(num,1);

for i=1:1:num
    rr(i)=sqrt(rx(i)^2+ry(i)^2);
    vv(i)=sqrt(vx(i)^2+vy(i)^2);
end

figure(1)
x=1:1:num;
plot(x,rx,'b-',x,vx,'r-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('v_x')     %添加图像标题
legend('期望值','实际值')
axis([0 7500 -1.5 1.5])

figure(2)
x=1:1:num;
plot(x,ry,'b-',x,vy,'r-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('v_y')     %添加图像标题
legend('期望值','实际值')
axis([0 7500 -1.5 1.5])

figure(3)
x=1:1:num;
plot(x,rr,'b-',x,vv,'r-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('速率')     %添加图像标题
legend('期望值','实际值')
axis([0 7500 0 1.5])