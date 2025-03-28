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

vx_b = vx + (max(vx)/40)*randn(12001,1);
vy_b = vy + (max(vy)/20)*randn(12001,1);
r_b = r + (max(r)/4)*randn(12001,1);

figure(100),
plot(t,vx_b,t,vy_b,t,r_b)

%% Modèle 3ddl

psir = zeros(length(t),1);
for i =2:length(t)
    psir(i,1) = euler2(r(i-1),psir(i-1,1),h);
end
vr = vy.*cos(psir(:,1)) - vx.*sin(psir(:,1));

%% Display

figure(1),
subplot(211),
plot(t,v,t,vr)
title('Vitesse v')
legend('mesurée','reconstruite')

subplot(212)
plot(t,psi,t,psir)
title('Angle psi')
legend('mesuré','reconstruit')

%% Influence du bruit

psir1 = psir;
vr1 = vy.*cos(psir1(:,1)) - vx_b.*sin(psir1(:,1));

psir2 = psir;
vr2 = vy_b.*cos(psir2(:,1)) - vx.*sin(psir2(:,1));

psir3 = zeros(length(t),1);
for i =2:length(t)
    psir3(i,1) = euler2(r_b(i-1),psir3(i-1,1),h);
end
vr3 = vy.*cos(psir3(:,1)) - vx.*sin(psir3(:,1));

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
title('Vitesse v avec bruit sur r')
legend('reconstruite','mesurée')

subplot(236)
plot(t,psir3,t,psir)
title('Angle psi avec bruit sur r')
legend('reconstruit','mesurée')

%%

psir4 = psir3;
vr4 =  vy_b.*cos(psir4(:,1)) - vx_b.*sin(psir4(:,1));

figure(4),
subplot(211),
plot(t,vr4,t,v)
title('Vitesse v')
legend('reconstruite','mesurée')

subplot(212)
plot(t,psir4,t,psi)
title('Angle psi')
legend('reconstruit','mesurée')