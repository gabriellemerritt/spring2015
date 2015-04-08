% [peaks, idx]=findpeaks(scope_A.signals.values(:,2));
clear peaksA peaksB indcA indcB Axft Bxft inAxft inBxft max_valA max_valA2 max_valB max_valB2 idxA idxB timeA timeB FsA FsB 
prompt = 'What is the value of the  A Amplitude? '; 
prompt2 = 'What is value of the B signal Amplitude '; 
timeA = scope_A.time; 
delt_tA = [delt_tA; ((timeA(end) - timeA(1))/length(timeA))]; 
FsA = 1/((timeA(end) - timeA(1))/length(timeA)); 
Axft = fft(scope_A.signals.values(:,2)); 
inAxft = fft(scope_A.signals.values(:,1)); 
Axft = Axft(2:end,:);
inAxft = inAxft(2:end,:);
[max_valA, idxA] = max(abs(Axft));  
[max_valA2, idxA2] = max(abs(inAxft)); 

wA = [wA;(FsA*(idxA-1)/length(scope_A.signals.values(:,2)))]; 
phaseA = [phaseA; unwrap(angle(inAxft(idxA2)),-pi) - unwrap(angle(Axft(idxA)),-pi)]; 
[peaksA,indcA] = findpeaks(scope_A.signals.values(:,2));
amp_A = [amp_A;peaksA(round(length(peaksA)/2))]; 
%%
timeB = scope_B.time; 
delt_tB = [delt_tB; ((timeB(end) - timeB(1))/length(timeB))]; 
FsB= 1/((timeB(end) - timeB(1))/length(timeB)); 
Bxft = fft(scope_B.signals.values(:,2)); 
inBxft = fft(scope_B.signals.values(:,1)); 
inBxft = inBxft(2:end,:); 
Bxft = Bxft(2:end,:); 
[max_valB, idxB] = max(abs(Bxft));  
[max_valB2, idxB2] = max(abs(inBxft)); 

wB = [wB;(FsB*(idxB-1)/length(scope_B.signals.values(:,2)))]; 
phaseB = [phaseB;unwrap(angle(inBxft(idxB2)),-pi) - unwrap(angle(Bxft(idxB)),-pi)]; 
[peaksB,indcB] = findpeaks(scope_B.signals.values(:,2));

amp_B=[amp_B;peaksB(round(length(peaksB)/2))]; 
%%

