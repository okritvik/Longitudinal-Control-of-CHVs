%Adaptive PIQ controller code with variable separation error gain(k) and
%0.2s actuator delay
clc;
clear all;
close all;
sl = 30; %initial Position of lead car
sf = 0; %initial position of following car
vf = 60; %initial velocity of following vehicle
vl = 40; %leading vehicle velocity
s0 = 5; %minimum distance required
h = 0.2; %fixed time headway
sd = s0+h*vf; %desired distance sd = so+hvf
xr = sl - sf; %initial relative distance
delta = xr-sd; %spacing error
vr = vl-vf; %following - leading velocities
vf_acc_list=[];
kp = 5;
ki = 0.5;
kq = 0.01;
a = 0.1;
b = 0.2;
figure
i=1;
for t = 0:0.1:50
    k=0.1+(1-0.1)*exp(-0.1*delta*delta);
    h = 0.1-0.2*vr; %for variable time headway 
    u = kp*(vr+k*delta) + ki + kq*(vr+k*delta)*abs(vr+k*delta); %u=kp(vr+k*delta) + ki + kq(vr+k*delta)|vr+k*delta|
    vf_acc = a*(vr+k*delta)+b*u;
    vf_acc_list(end+1) = vf_acc;
    %comment the if condition (32-35 lines) to get simulation with no
    %actuator delay and uncomment line 36
%     if(t>0.1)
%         vf = vf+(vf_acc_list(i)*0.1);
%         i=i+1;
%     end
    vf = vf+vf_acc*0.1;
    subplot(2,2,1)
    plot(t,vf,'.','Color','red');
    xlabel('time')
    ylabel('m/s')
    title('Following Vehicle Velocity')
    xlim([0 50])
    ylim([0 60])
    hold on
    drawnow
    subplot(2,2,2)
    plot(t,delta,'.','Color','blue');
    xlabel('time')
    ylabel('m')
    title('Separation Error')
    xlim([0 50])
    ylim([-250 30])
    hold on
    drawnow
    subplot(2,2,3)
    plot(t,vf_acc,'.','Color','black');
    xlabel('time')
    ylabel('m/s2')
    title('Following Vehicle Acceleration')
    xlim([0 50])
    ylim([-30 30])
    hold on
    drawnow
    subplot(2,2,4)
    plot(t,xr,'.','Color','m');
    xlabel('time')
    ylabel('m')
    title('Vehicle Separation')
    xlim([0 50])
    ylim([-30 60])
    hold on
    drawnow
    sd = s0+h*vf;
    if(t>2 && t<4.1)%accelerating lead vehicle with 2m/s^2 for 2 seconds
        vl = vl+2*0.1;
    end
    vr = vl-vf;
    sl = sl+vl*0.1;
    sf = sf + vf*0.1;
    xr = sl-sf;
    delta = xr-sd;
end 