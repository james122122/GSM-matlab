% MMI 502 Lab 7 - Seth Hochberg
% https://github.com/sethhochberg/matlab_granular_synthesis/blob/master/Lab7.m
% break granular synth into steps
% framesize/grainsize 1/20 why?
% use same numframes~~~~
% 1 - numframes

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
%sample rate accordingly..
%signal = signal(find(signal,1,'first'):find(signal,1,'last'));
%signal2 = signal2(find(signal2,1,'first'):find(signal2,1,'last'));

framesize = .05 * fs; % 1 frame = 1/20 sample rate
hopsize = round(.5 * framesize); %hopsize = half framesize

startpos = 1;
endpos = length(signal)-framesize;

startpos2 = 1;
endpos2 = (length(signal2)-framesize)/3;

outputLengthInput = 400000;
outputLength = ceil(outputLengthInput/framesize) * framesize;
grainSpace = 500;
grainSpace2 = 500;

numframes = floor(((endpos + hopsize) / hopsize)); %total samples - frame
numframes2 = floor(((endpos2 - framesize + hopsize) / hopsize) / 3);

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

%randframes = randperm(numframes); % create random numbers which sequece from first one (1) to number of frames: numframes
for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        % newsignal(signalindex) = newsignal(signalindex) + framematrix(:,randframes(currentframe))';
        
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)';
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

newsignalRepeat = repmat(newsignal,1,ceil(outputLength/length(newsignal)));
newsignalRepeat = newsignalRepeat(:,1:outputLength);

newsignalRepeat2 = repmat(newsignal2,1,ceil(outputLength/length(newsignal2)));
newsignalRepeat2 = newsignalRepeat2(:,1:outputLength);

figure;
subplot(211);
plot(signal);
title('Original');

subplot(212);
plot(newsignalRepeat);
title('Random');

soundsc(newsignalRepeat, fs);

pause(4);

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