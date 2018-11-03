clc
clear all
close all

Archivo1='datos-senal.xlsx';

senal = input('Senal a analizar 3 o 11? ');
frec_fundamental = input('Frecuencia fundamental: ');
I_l = input('IL para TDD: ');
N_ciclos_tot = input('Ciclos a graficar : ');
comp_CD = input('Componente de directa: ');
V_factor_p = input('Nivel de tension: ');
defase_VI = input('Angulo entre V y I: ');
V_rms = V_factor_p / sqrt(2);

mc = 256;
fm = mc * frec_fundamental;
tm = 1/fm;
t = 0:tm:N_ciclos_tot/frec_fundamental;
i=1;
S3_tot = 0;

if senal == 3;
  hoja = 'Senal3';
else
  hoja = 'Senal11';
end

  
arms = xlsread(Archivo1,hoja,'B1:B180').'
ampls = xlsread(Archivo1,hoja,'A1:A180').'
angulos = xlsread(Archivo1,hoja,'C1:C180').'
w_ieee519 = xlsread(Archivo1,hoja,'D1:D180').'

in_senales = sprintf('Hasta cuantos de los %d armonicos quiere evaluar: ', length(arms)); 
senales = input(in_senales);

while (i <= senales)
  funcion = ampls(i)*sin(arms(i)*2*pi*frec_fundamental*t + deg2rad(angulos(i)));
  subplot(2,1,1);
  plot(t,funcion);
  S3_tot = S3_tot + funcion;  %Para ver la evolucion de la funcion resultante.
  subplot(2,1,2);
  plot(t,S3_tot);
  i = i + 1;
  pause(0);
end

S3_tot_CD = comp_CD + S3_tot;

vec_ampls = ampls / sqrt(2); %rms
ampl_rms_fundamental = ampls(1) / sqrt(2);  
ampl_rms_total = vec_ampls / sqrt(2); % vector de RMS's de armonicos
ampl_l_fundamental = ampls(1);
%vector_IHD = ampl_rms_total / ampl_rms_fundamental; 
vector_IHD = ampls / ampl_l_fundamental; % rms o pico ?  --> rms


amplitudes = length(vec_ampls);
rms_distorcion = 0;
tif_arms = 0;
i = 2;  %dejando fuera la fundamental


while (i <= amplitudes)
  mag_2 = vec_ampls(i)^2;
  tif_n = mag_2 * w_ieee519(i)^2;
  rms_distorcion = rms_distorcion + mag_2;
  tif_arms = tif_arms + tif_n;
  i = i + 1;
end

k_ampls = 0;
k_ampls_arm = 0;
rms_total = 0;
j = 1;
while (j <= amplitudes)
  ampl_rms = vec_ampls(j)^2;
  ampl_k = vector_IHD(j)^2;
  ampl_k_arm = ampl_k * arms(j)^2;
  k_ampls = k_ampls + ampl_k;
  k_ampls_arm = k_ampls_arm + ampl_k_arm;
  rms_total = rms_total + ampl_rms;
  j = j + 1;
end

ampl_rms_fundamental
%ampl_rms = ampl_rms_total
rms_total = sqrt(rms_total)
THD = sqrt(rms_distorcion) / ampl_rms_fundamental
TDD = sqrt(rms_distorcion) / I_l
TIF = sqrt(tif_arms) / ampl_rms_fundamental
AMP_por_TIF = TIF * ampl_rms_fundamental
Factor_K = k_ampls_arm / k_ampls

%pot_real = 

%PF_total = 

hold off

%in_save_ws = input('Nombre de archivo para Workspace (Entre comillas, sin extension): ');
%save_ws = sprintf('%s.mat', in_save_ws);
%save ("-mat7-binary", save_ws, "mc", "S3_tot_CD", "t");

%clc
%clear all
%close all

  