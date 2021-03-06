% Based on MMI 502 Lab 7 - Seth Hochberg
% https://github.com/sethhochberg/matlab_granular_synthesis/blob/master/Lab7.m

%speeds, phases, volume, and frequency,

%%%%%%%%% Grain File Offset %%%%%%%%%%%%%
%startpos = ?;
%startpos2 = ?;

%%%%%%%%% Grain length/duration %%%%%%%%%%%%%%
% Must be less than output length
% Need to repeat this grain [in random orders?] to extend to entire output length

%endpos = startpos+customLength(signal)-framesize (minus one frame)
%endpos must be <= outputLength

%endpos2 = startpos2+customLength(signal2)-framesize (minus one frame)

%%%%%%%%% Output Length %%%%%%%%%%%%%
%implementing length variations
%outputLength = ?

%%%%%%%%%% Grain spacing %%%%%%%%%%%%%% I.E. inter-onset time
%%%%%%%%%%may be set to be random
% Add zeros of size x in between grains


%%%%%%%%%%%%%%%%
% Envelope / Window Shape / grain ADSR
% triangular...

%%%%%%%%%% Grain density %%%%%%%%%%%%%
% affects pitch~~~~
% How many grains per second - this will be a function of grain
% duration,spacing, and output length

%%%%%%%%%% Post-grain attenuation %%%%%%%%%%%%
%   Spray
%   randomises the grain start position. The spray parameter
%   determines how far from the start position the sound can possibly move
%   
%   Amplitude Modulation
%   Controls the gain (amplitude) of each grain via a random number multiplier/function
%
%   




% today - granulator functional with basic parameters
% 17/9 - theory of granular and set of (attenuation) parameters for both
% grain manipulation and output
% 21/9 implement 3x attenuation/granular parameters on granular synthesis
% 28/9 parameter interpolation - linear + non-linear algorithm etc.
% 5/9 research more algorithms + explore another method of interpolation/algorithm
% frequency pitch .. energy shifting .. energy distribution
% 12/9 

%% Problem 1 - Import, make granular, recombine, export
[signal fs bitdepth] = wavread('violin-a4.wav'); % [sample data - sample rate - number of bits per sample]
[signal2 fs bitdepth] = wavread('trumpet.wav'); % [sample data - sample rate - number of bits per sample]

%need to trim zeros from beginning + end when reading signals AND adjust
%if < +/-0.005, trim
signal = signal( signal(:,1) < -0.0002 | signal(:,1) > 0.0002, :);
signal2 = signal2( signal2(:,1) < -0.0002 | signal2(:,1) > 0.0002, :);
figure;plot(signal);
figure;plot(signal2);

% audioplayer is better, will need to implement 'playblocking' to ensure
% only one sound plays at a time.
player = audioplayer(signal,fs);
play(player);

%soundsc(signal, fs);
pause(4);
soundsc(signal2, fs);
pause(4);

framesize = .05 * fs; % 1 frame = 1/20 sample rate
hopsize = round(.5 * framesize); %hopsize = half framesize

startpos = 1;
endpos = floor(length(signal)-framesize);

startpos2 = 1;
endpos2 = floor((length(signal2)-framesize)/3);

outputLengthInput = 400000;
outputLength = ceil(outputLengthInput/framesize) * framesize;

grainSpace = 500;
grainSpace2 = 500;

numframes = floor(((endpos-startpos - framesize + hopsize) / hopsize)); %total samples - frame
numframes2 = floor((endpos2-startpos2 - framesize + hopsize) / hopsize);

framematrix = zeros(framesize,outputLength);
framematrix2 = zeros(framesize,outputLength);

%Decompose signal into frames
for currentframe = startpos:hopsize:endpos %each frame is one [hopsize] from 1 to last sample-1frame
        thisframe = signal(currentframe:currentframe+framesize-1,1); %frame = the signal from 1 to framesize-1 then framesize to 2framesize-1....
        %thisframe = thisframe';
        framematrix(:,round(currentframe/hopsize)+1) = thisframe .* hanning(framesize); %this part was changed%%%
        %changed thisframe to (:,1) as each side was conflicting sizes
        %plot(thisframe' .* hanning(framesize));
        %pause;
end


for currentframe = startpos2:hopsize:endpos2 %each frame is one [hopsize] from 1 to last sample-1frame
        thisframe = signal2(currentframe:currentframe+framesize-1,1); %frame = the signal from 1 to framesize-1 then framesize to 2framesize-1....
        framematrix2(:,round(currentframe/hopsize)+1) = thisframe .* hanning(framesize); %this part was changed%%%
        %changed thisframe to (:,1) as each side was conflicting sizes
        %plot(thisframe' .* hanning(framesize));
        %pause;
end

%% Reconstructing the signal in random grain order
newsignal = zeros(startpos, endpos+framesize);
newsignal2 = zeros(startpos2, endpos2+framesize);

randframes = randperm(numframes); % create random numbers which sequece from first one (1) to number of frames: numframes
for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        
        %newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)';
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,randframes(currentframe))';
        
        % newsignal --- signal of length numframes
        % signalindex --- index of frames from 1 to framesize, framesize+1
        % to 2xframesize
        % newsignalRepeat --- signal of length outputLength
end % mod used to repeat signal...

%randframes2 = randperm(numframes2); % create random numbers which sequece from first one (1) to number - numframes
for currentframe = 1:numframes2
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        newsignal2(signalindex) = newsignal2(signalindex) + framematrix2(:,(numframes2-currentframe)+1)';
end

%remove trailing zeros from newsignals
newsignal = newsignal(newsignal~=0); 
newsignal2 = newsignal2(newsignal2~=0);

figure;plot(newsignal);
figure;plot(newsignal2);

newsignalRepeat = repmat(newsignal,1,ceil(outputLength/length(newsignal)));
newsignalRepeat = newsignalRepeat(:,1:outputLength);

newsignalRepeat2 = repmat(newsignal2,1,ceil(outputLength/length(newsignal2)));
newsignalRepeat2 = newsignalRepeat2(:,1:outputLength); %error in this line after adding line 128 - nonzeros(newsignal2)

figure;
subplot(211);
plot(signal);
title('Original');

subplot(212);
plot(newsignalRepeat);
title('Random');

%soundsc(newsignalRepeat, fs);
soundsc(newsignal, fs);

pause(8);

figure;
subplot(211);
plot(signal2);
title('Original');

subplot(212);
plot(newsignalRepeat2);
title('Random');

soundsc(newsignalRepeat2, fs);

pause(4);

%add two signals together...
%{
for i = 1:length(newsignal)
       newsignal2(:,i) = (newsignal2(:,i) + newsignal(:,i)) / 2;
end
%}


%wavwrite(newsignal, fs, 'trumpted_recombined_randomized.wav');

%{
%% Reconstruct the signal in normal order, unmodified
newsignal = zeros(1, length(signal));
for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,currentframe)';
end

%{
figure;
subplot(211);
plot(signal);
title('Original');
subplot(212);
plot(newsignal);
title('Recomposed');
%}
soundsc(newsignal, fs);

wavwrite(newsignal, fs, 'trumpted_recombined.wav');
%}

%{

%% Reconstructing the signal in the reverse order
newsignal = zeros(1, length(signal));
newsignal2 = zeros(1, length(signal2));

for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)'; %starts from last
end
%{
figure;
subplot(211);
plot(signal);
title('Original');
subplot(212);
plot(newsignal);
title('Reversed');
%}
soundsc(newsignal, fs);
wavwrite(newsignal, fs, 'trumpted_recombined_reversed.wav');
%}