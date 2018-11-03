%%
clc
clear all
close all

FI=menu('Eventos de Analisis','SENAL A ANALIZAR');
%%

if FI==1;
  load('signal_11_ok.mat');
Ia=S11_tot_CD;
t=t

fo=input('Frecuencia fundamental:  ');   %Frecuencia de señal
mc_input=mc;                  %% Resolucion de señal de entrada
Tm=1/(mc_input*fo);        %100e-6;
fm=1/Tm;   % Muestras por segundo
fmc=fm/fo; % Muestras por ciclo


%%
figure(99)
plot(t,Ia,'-r'); hold on; 
title('Señales Analizadas de Corriente');
xlabel('Time (sec)')
ylabel('Amplitude')

Fmc=round(fmc);                           % Frecuencia de muestreo
wo=fo*2*pi;

%% Muestras por ciclo
display('Muestras por ciclo')
mc=Fmc
nc=length(Ia); 
freq=fo;           %% Frecuencia fundamental del sistema

%% Señales de voltaje y corriente

display('%% Señales continuas Ia,Ib,Ic %%')

%yit=input('Señal de corriente a analizar:    ');  %(577:end);   %% Ia sera la señal de corriente a analizar
yit = Ia;

%% Vector de tiempo %%
longi=length(yit)           %% cantidad de datos de senal
t=(0:longi-1)*(1/(mc*freq));
t=t.';                       %(1:962);%tnew;%t; t2%T1;%
ttot=t;
%%
e=2*pi*fo;   
%%
figure(1)
plot(t,yit,'r')
title('Current Signal (FULL)')
xlabel('time, sec')
ylabel('Amplitude')

print -djpg figura1.jpg

%%
% ANALISIS ESPECTRAL FFT %%
% No de ciclos de analisis 

display ('%% Numero Total de ventanas de señal %%')
NT=nc/mc
Vent1=input('Numero de ventana de señal a analizar INICIAL:   ');
Vent=input('Numero de ventana de señal a analizar FINAL:   ');     %% No. de ciclos a analizar

% REVISAR %%
yi1=yit(Vent1:mc*Vent);

t=t(Vent1:mc*Vent);

% Señal de analisis
figure(3)
plot(t,yi1,'r');     
title(' Current, ANALYZED SIGNAL')
xlabel('Time, (sec)')
ylabel('Amplitude (A)')
h=legend('I_r','Location','Best');  

print -djpg figura3.jpg

%%
% Señales electricas de entrada %%
I2=yi1.';
t2=t.';
T=Tm;%1/(Fmc*fo); % revisar si funciona con T o Tm.
Fs=Fmc*fo;

%
%%
% $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FFT Algorithm  (CURRENT)
  
L1=length(I2)         % Length of signal

t11=(0:L1-1)*T;                  % Time vector
                                   % I2, Your CP values go in this vector
%asdasd
% RESOLUCION FFT POINTS %%
% PARA FINES COMPARATIVOS EXPLORER MATLAB %%

NFFT1 =0.56*(L1)%length(yit1)*2; %    =0.56*(L1)   2^nextpow2(L1)            2^nextpow2*(L); Next power of 2 from length of y
%NFFT1 = (L1/NT)*Vent
%NFFT1 = L1
Yi1=fft(I2,NFFT1)/NFFT1;
absYi=2*abs(Yi1(1:NFFT1/2+1));

fi = linspace(0,Fs/2,NFFT1/2+1);
%fi = (Fs/2)*linspace(0,1,NFFT/2+1);

fn1=fi;
% PARA FINES DE ANALISIS DE SEÑALES SELECCION DE SEÑAL %%
% NFFT = 2^nextpow2*(L1); %Next power of 2 from length of y
% Yi1=fft(I2,NFFT1)/L1;
% absYi=2*abs(Yi1(1:NFFT1/2+1));
% 
% fi = linspace(0,Fs/2,NFFT1/2+1);
% %fv = (Fs/2)*linspace(0,1,NFFT/2+1);
% 
% fn1=fi;
%
norm_dataI=((Yi1))./max(((Yi1)));
Psd_est_dataI=(abs(norm_dataI(1:NFFT1/2+1)).^2);
Edbi=10*log10(Psd_est_dataI);
EFi=fi;
% HV=spectrum.periodogram;
% Psd_estV=psd(HV,V2,'Fs',Fs);
% Psd_est_dataV=Psd_estV.Data;
% Psd_est_dataV=Psd_est_dataV./max(Psd_est_dataV);
%
figure(7)
% EFv=Psd_estV.Frequencies;
% Edbv=10*log10(Psd_est_dataV);
% plot(EFv,Edbv,'-b')

plot(EFi,Edbi,'-r')
grid on
title('Periodogram Using FFT for Current SIGNAL')
xlabel('Frequency (Hz)')
ylabel('Power (dB)')
%
% clear Psd_est.Frequencies Psd_est.Data
% 
% HI=spectrum.periodogram;
% Psd_estI=psd(HI,I2,'Fs',Fs);
% Psd_est_dataI=Psd_estI.Data;
% Psd_est_dataI=Psd_est_dataI./max(Psd_est_dataI);
% %%
% figure(7)
% plot(Psd_estI.Frequencies,10*log10(Psd_est_dataI),'-r')
% grid on
% title('Periodogram Using FFT for CURRENT SIGNAL')
% xlabel('Frequency (Hz)')
% ylabel('Power/Frequency (dB/Hz)')
%

xipre=fn1;
yipre=absYi;
%ydb = mag2db(y)
%

print -djpg figura7.jpg

%hold on
figure(8)
plot(xipre,yipre,'r')
title(' Frequency spectrum current signal (Ir) ')
xlabel('Frequency, Hz')
ylabel('Magnitude, A')

print -djpg figura8.jpg

% MAIN PLOTS %% 

%% PENDIENTE
figure(9);
subplot(2,1,1);
plot(t,I2,'-r'); hold on; h = legend('Corriente');
title('Señal de Corriente Analizada');
xlabel('Time (sec)')
ylabel('Amplitude (A)')

subplot(2,1,2)
plot(xipre,yipre,'-r'); 
title('ESPECTROS DE CORRIENTE (LINEAL)');
xlabel('Frequency, Hz')
ylabel('Magnitude, A')
xipre_t = transpose(xipre);
save ("datos11.txt", "xipre_t", "yipre");

print -djpg figura9.jpg


%% COMPARACION DE Espectro en dB y lineal y compararlo para cada evento %%
figure(10);

subplot(3,1,1);
plot(t,I2,'-r'); hold on; h = legend('Corriente');
title('Señal de Corriente Analizada');
xlabel('Time (sec)')
ylabel('Amplitude (A)')

subplot(3,1,2)
plot(xipre,yipre,'-r'); 
title('ESPECTROS DE CORRIENTE (LINEAL)');
xlabel('Frequency, Hz')
ylabel('Magnitude, A')

subplot(3,1,3)
plot(EFi,Edbi,'-r');
title('ESPECTROS DE CORRIENTE (dB)');
xlabel('Frequency, Hz')
ylabel('Power (dB)')

print -djpg figura10.jpg


    
end




    