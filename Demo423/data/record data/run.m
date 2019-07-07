vs1=importdata("vs1.txt");
rs1=importdata("rs1.txt");
num1=length(vs1);
rs1=[0 0;rs1];

vs2=importdata("vs2.txt");
rs2=importdata("rs2.txt");
num2=length(vs2);
rs2=[0 0;rs2];

rx1=rs1(:,1);
ry1=rs1(:,2);
rx2=rs2(:,1);
ry2=rs2(:,2);


vx1=vs1(:,1);
vy1=vs1(:,2);
vx2=vs2(:,1);
vy2=vs2(:,2);

num1=length(vs1);

rr1=zeros(num1,1);
vv1=zeros(num1,1);

for i=1:1:num1
    rr1(i)=sqrt(rx1(i)^2+ry1(i)^2);
    vv1(i)=sqrt(vx1(i)^2+vy1(i)^2);
end
for i=1:1:num2
    rr2(i)=sqrt(rx2(i)^2+ry2(i)^2);
    vv2(i)=sqrt(vx2(i)^2+vy2(i)^2);
end

figure(1)
x=1:1:num1;
x2=1:1:num2;
plot(x,rx1,'b-.',x,vx1,'r-',x2,rx2,'k-.',x2,vx2,'g-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('v_x')     %添加图像标题
legend('无人机1期望值','无人机1实际值','无人机2期望值','无人机2实际值')
axis([0 3500 -1.5 1.5])

figure(2)
x=1:1:num1;
x2=1:1:num2;
plot(x,ry1,'b-.',x,vy1,'r-',x2,ry2,'k-.',x2,vy2,'g-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('v_y')     %添加图像标题
legend('无人机1期望值','无人机1实际值','无人机2期望值','无人机2实际值')
axis([0 3500 -1.5 1.5])

figure(3)
x=1:1:num1;
x2=1:1:num2;
plot(x,rr1,'b-.',x,vv1,'r-',x2,vv2,'g-');
grid on
xlabel('迭代数')
ylabel('m/s')
title('速率')     %添加图像标题
legend('速率期望值','无人机1实际值','无人机2实际值')
axis([0 3500 0 1.5])