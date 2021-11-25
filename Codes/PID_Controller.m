%PID controller without actuator delay of 0.2ms
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
past_vf =  vf;
kp = 1.5;
ki = 0.3;
kd = 0.01;
vf_acc_list=[];
i=1;
figure
for t=0:0.1:50
    k=0.1+(1-0.1)*exp(-0.1*delta*delta);
    h = 0.1-0.2*vr; %for variable time headway - comment this line for 
    u = kp*(vr+k*delta) + ki*(vr+k*delta)*0.1 + kd*(vr+k*delta)*((vf-past_vf)/0.1); %u = kp*(vr+k*delta)+ki*(1/s)(vr+k*delta)+kd*(s/sTd+1)*(vr+k*delta): Td is small and discrete differentiation (dv/dt)
    vf_acc = 0.1*(vr+k*delta)+0.2*u; %vfdot = a*(vr+k*delta)+b*u
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
    past_vf = vf;
    vf_acc_list(end+1) = vf_acc;
    %comment the if condition (36-39 lines) to get simulation with no
    %actuator delay and uncomment line 40
    if(t>0.1)
        vf = vf+(vf_acc_list(i)*0.1);
        i=i+1;
    end
%     vf = vf+vf_acc*0.1;
    sd = s0+h*vf;
    if(t>2 && t<4.1) %accelerating lead vehicle with 2m/s^2 for 2 seconds 
        vl = vl+2*0.1;
    end
    vr = vl-vf;
    sl = sl+vl*0.1;
    sf = sf + vf*0.1;
    xr = sl-sf;
    delta = xr-sd;
end