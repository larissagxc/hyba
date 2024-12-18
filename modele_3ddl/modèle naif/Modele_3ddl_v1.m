load('data2.mat')

T_final = 600;	       
h = 0.05; 
t = 0:h:T_final;  

u     = simdata(:,1); 
v     = simdata(:,2);          
p     = rad2deg(simdata(:,3));   
r     = (simdata(:,4));
phi   = rad2deg(simdata(:,5));
psi   = simdata(:,6);
psid   = psi*180/pi;
tauN  = simdata(:,7);
U     = simdata(:,8);
x     = simdata(:,9);
y     = simdata(:,10);
psi_d = rad2deg(simdata(:,11)); % Commande de l'angle psi sur la simulation

vx = u.*cos(psi)-v.*sin(psi);
vy = u.*sin(psi)+v.*cos(psi);

vx2 = gradient(x,h);
vy2 = gradient(y,h);

% figure(10),
% subplot(211),
% plot(t,vx,t,vx2)
% title('Vitesse vx')
% legend('rotation','gradient')
% 
% subplot(212),
% plot(t,vy,t,vy2)
% title('Vitesse vy')
% legend('rotation','gradient')

vx_b = vx + (max(vx)/400)*randn(12001,1);
vy_b = vy + (max(vy)/200)*randn(12001,1);
u_b = u + (max(u)/400)*randn(12001,1);

figure(100),
plot(t,vx_b,t,vy_b,t,u_b)

%% Modèle 3ddl

vr = -sqrt(abs(vx.^2+vy.^2-u.^2));
psir = atan2(vy,vx)-atan2(vr,u);

%% Display

figure(10),
subplot(211),
plot(t,v,t,vr)
title('Vitesse v')
legend('mesurée','reconstruite')

subplot(212)
plot(t,psi,t,psir)
title('Angle psi')
legend('mesuré','reconstruit')

%% Influence du bruit

vr1 = -sqrt(abs(vx_b.^2+vy.^2-u.^2));
psir1 = atan2(vy,vx_b)-atan2(vr1,u);

vr2 = -sqrt(abs(vx.^2+vy_b.^2-u.^2));
psir2 = atan2(vy_b,vx)-atan2(vr2,u);

vr3 = -sqrt(abs(vx.^2+vy.^2-u_b.^2));
psir3 = atan2(vy,vx)-atan2(vr3,u_b);
beta3 = atan2(vr,u_b);

figure(2),
subplot(231),
plot(t,vr1,t,v)
title('Vitesse v avec bruit sur vx')
legend('reconstruite','mesurée')

subplot(234)
plot(t,psir1,t,psi)
title('Angle psi avec bruit sur vx')
legend('reconstruit','mesurée')

subplot(232),
plot(t,vr2,t,v)
title('Vitesse v avec bruit sur vy')
legend('reconstruite','mesurée')

subplot(235)
plot(t,psir2,t,psi)
title('Angle psi avec bruit sur vy')
legend('reconstruit','mesurée')

subplot(233),
plot(t,vr3,t,v)
title('Vitesse v avec bruit sur u')
legend('reconstruite','mesurée')

subplot(236)
plot(t,psir3,t,psir)
title('Angle psi avec bruit sur u')
legend('reconstruit','mesurée')

%%

vr4 = -sqrt(abs(vx_b.^2+vy_b.^2-u_b.^2));
psir4 = atan2(vy_b,vx_b)-atan2(vr4,u_b);

figure(4),
subplot(211),
plot(t,vr4,t,v)
title('Vitesse v')
legend('reconstruite','mesurée')

subplot(212)
plot(t,psir4,t,psi)
title('Angle psi')
legend('reconstruit','mesurée')