% Sylvain lanneau
% 18 juillet 2024
% Trace les données brutes d'un essai. Essais définis dans
% MetadataEssais.xlsx

% ici il est nécessaire créer un nouveau archive %Metadonne_qqch
% et puis l'ajouter dans le code "importNaonedData.m" -> f = fullfile()
% peut-être changer le code qui trouve le folder (getDataFichier.m)

clc
clear
close all

% Récupération des répertoires des fonctions et data dans le fichier de
% config
config = importdata('Config.txt');
s = split(config{1},"=");
path_CODES = s{2};
s = split(config{2},"=");
path_DATA = s{2};
addpath(genpath(path_CODES));

% Identifiant de l'essai dont les données sont à tracer
id_essai = "petits_zigzags";

% Import des données brutes dans 2 tables, 10Hz et 1Hz
[data10,data1,metadata] = importNaonedData(path_DATA,id_essai);

% Affichage des noms des signaux contenus dans chaque table
data10.Properties.VariableNames
data1.Properties.VariableNames

% Affichage des unités des signaux contenus dans chaque table
data10.Properties.VariableUnits
data1.Properties.VariableUnits

% Sous-échantillonnage 1Hz et concaténation
data = sousEchEtConcat(data10,data1);

% Vecteurs d'horotadages
hd1 = datetime(data1.("PLC_AMS_23084_ECOMER v0.8"),'InputFormat','dd/MM/yyyy - HH:mm:ss.S');
hd10 = datetime(data10.("PLC_AMS_23084_ECOMER v0.8"),'InputFormat','dd/MM/yyyy - HH:mm:ss.S');

% Reconstitution du temps en secondes pour les tracés
t10 = linspace(0,0.1*height(data10),height(data10));
t1 = linspace(0,height(data1),height(data1));


%%
figure('WindowState', 'maximized');sgtitle(id_essai)

subplot(2,3,1)
g = geoplot(data1.("GPS LATITUDE"),data1.("GPS LONGITUDE"));
hold on
geoplot(data1.("GPS LATITUDE")(1),data1.("GPS LONGITUDE")(1),'r*')
text(data1.("GPS LATITUDE")(1),data1.("GPS LONGITUDE")(1),'Départ','color','r')
g.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow("Horodatage",data1.("PLC_AMS_23084_ECOMER v0.8"));
grid on


subplot(2,3,2)
plot(hd10,data10.("PAS D HELICE"),hd10,data10.("ANGLE DE BARRE"))
hold on
%plot(hd10,data10.("REGULATEUR DE VITESSE"))
legend('Pas de l''hélice','Angle de barre')
title('')
xlabel('Temps (s)')
grid on

subplot(2,3,3)
plot(hd1,data1.("VITESSE FOND GPS"),hd1,data1.("VITESSE SURFACE LOCH"))
legend('Vitesse fond GPS','Vitesse loch')
title('')
xlabel('Temps (s)')
ylabel('noeuds')
grid on

subplot(2,3,4)
plot(hd10,data10.("ROTATIONAL SPEED Z AXIS"))
legend('Vitesse angulaire Z')
title('')
xlabel('Temps (s)')
ylabel('deg/s')
grid on

subplot(2,3,5)
plot(hd10,data10.("ACCELERATION X AXIS"))
hold on
plot(hd10,data10.("ACCELERATION Y AXIS"))
legend('Accel X','Accel Y')
title('')
xlabel('Temps (s)')
ylabel('(m/s^2)')
grid on

conso = calculConso(data10.("IMPULSIONS DEBITMETRE MP ALLER"),data10.("IMPULSIONS DEBITMETRE MP RETOUR"),height(data1));

subplot(2,3,6)
plot(hd10,data10.("REGIME MOTEUR"))
hold on
plot(hd1,conso)
legend('Régime moteur','Consommation')
title('')
xlabel('Temps (s)')
ylabel('(rpm), (l/h)')
grid on

