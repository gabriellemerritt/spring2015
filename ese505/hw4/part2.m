
num = [1000]; 
den = [1 200]; 

G = tf(num,den);
w = logspace(-1,10,1000); % learn the logspace command - set desired range
[mag,pha]=bode(G,w);        % bode gives you magnitude and phase
mag = squeeze(mag); pha=squeeze(pha); 
figure(1);
set(gcf,'Color','w');
subplot(211);
loglog(w,mag,'-b','LineWidth',3); grid on; hold on;
%  axis([0.1 500 0.1 40]);
ylabel('Magnitude');
%
subplot(212);
semilogx(w,pha,'-r','LineWidth',3); grid on; hold on;
%  axis([0.1 500 -180 0]);
set(gca,'YTick',[-180 -135 -90 -45 0]);
ylabel('Phase [deg]');
xlabel('Frequency [rps]');