% cade02_v4 genera grafico de la funcion resultante en el dominio del
% tiempo conociendo los valores en el dominio de la frecuencia
% ...(Ademas me sirvio para aprender Octave / Matlab)...
%
% Creado por Jesus Guajardo 0455051
% MIOE
% FIME - UANL
%
% Se requiere funcionalidad para leer informacion de archivos de excel
% En octave se requiere paquete "I/O".
% Instalar mediante:
%
%pkg install -forge io
%
% Para cargar (una vez instalado):
%
%pkg load io

clc
clear all
close all

pkg load io; %Para octave

Archivo1='datos-senal.xlsx';

%senal = input('Senal a analizar 3 o 11? ');
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


hoja1 = 'Senal3';
hoja2 = 'Senal11';


  
arms = xlsread(Archivo1,hoja1,'B1:B180').';
ampls = xlsread(Archivo1,hoja1,'A1:A180').';
angulos = xlsread(Archivo1,hoja1,'C1:C180').';
w_ieee519 = xlsread(Archivo1,hoja1,'D1:D180').';

arms2 = xlsread(Archivo1,hoja2,'B1:B180').';
ampls2 = xlsread(Archivo1,hoja2,'A1:A180').';
angulos2 = xlsread(Archivo1,hoja2,'C1:C180').';
w_ieee519_2 = xlsread(Archivo1,hoja2,'D1:D180').';

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
vec_ampls2 = ampls2 / sqrt(2); %rms
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
pot_real = 0;
pot_reac = 0;
j = 1;
while (j <= amplitudes)
  ampl_rms = vec_ampls(j)^2;
  ampl2_rms = vec_ampls2(j)^2;
  p_real = vec_ampls(j) * vec_ampls2(j) * cos(angulos(j) - angulos2(j));
  p_reac = vec_ampls(j) * vec_ampls2(j) * sin(angulos(j) - angulos2(j));
  ampl_k = vector_IHD(j)^2;
  ampl_k_arm = ampl_k * arms(j)^2;
  k_ampls = k_ampls + ampl_k;
  k_ampls_arm = k_ampls_arm + ampl_k_arm;
  rms_total = rms_total + ampl_rms;
  pot_real = pot_real + p_real;
  pot_reac = pot_reac + p_reac;
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
pot_real
pot_reac


  

%pot_real = 

%PF_total = 

hold off

%in_save_ws = input('Nombre de archivo para Workspace (Entre comillas, sin extension): ');
%save_ws = sprintf('%s.mat', in_save_ws);
%save ("-mat7-binary", save_ws, "mc", "S3_tot_CD", "t");

%clc
%clear all
%close all

  