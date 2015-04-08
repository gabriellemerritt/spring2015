%
% 2015-HW04
% XXX = stuff you have to fill in
%
close all;
%
% put guess at transfer function here
% include appropriate range of frequencies in logspace command
%
freq_A = .6357;
damping_A = 0.1216;
% num = .46*[1]     % you have to figure out num & den to match data
num = [900       24150      183000      607500      750000]
% den = [1 0 .46];
den =  [ 6         167        1490        6329       12988 10100];
G = tf(num,den);
w = logspace(-1,10,1000); % learn the logspace command - set desired range
[mag,pha]=bode(G,w);        % bode gives you magnitude and phase
mag = squeeze(mag); pha=squeeze(pha);   % convert output to simple vectors
%
% observations from simulink here
% suggestion: make observations about time between peaks of input
% and output waves then convert those to phase change
%
wdata = wB;
magdata = amp_B;
delta_t_data = delt_tB;
phadata = test2;
%
% make pretty plots of the frequency response
%
figure(1);
set(gcf,'Color','w');
subplot(211);
loglog(w,mag,'-k','LineWidth',2); grid on; hold on;
loglog(wdata,magdata,'or','MarkerSize',8,'MarkerFaceColor','r');
axis([0.1 10 0.1 10]);
ylabel('Magnitude');
%
subplot(212);
semilogx(w,pha,'-k','LineWidth',2); grid on; hold on;
semilogx(wdata,phadata,'or','MarkerSize',8,'MarkerFaceColor','r');
axis([0.1 10 -180 0]);
set(gca,'YTick',[-180 -135 -90 -45 0]);
ylabel('Phase [deg]');
xlabel('Frequency [rps]');