%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Granular Synthesis - Morphing Script
% UTS Capstone Project 2018
% Student: James Dolphin
% Supervisor: Dr. Eva Cheng
% Base dervied from MMI 502 Lab 7 - Seth Hochberg
% https://github.com/sethhochberg/matlab_granular_synthesis/blob/master/Lab7.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
%   randomises the grain start position, and therefore length. The spray parameter
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
signal = remove_low_vals(signal);
signal2 = remove_low_vals(signal2);

% audioplayer is better, will need to implement 'playblocking' to ensure
% only one sound plays at a time.
player = audioplayer(signal,fs);
playblocking(player);

player = audioplayer(signal2,fs);
playblocking(player);

%%%%%%%%%%%%%%%%%%%%%%   input variables  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

framesize = .05 * fs; % 1 frame = 1/20 sample rate
hopsize = round(.5 * framesize); % hopsize = half framesize

%startpos and endpos still have not been tested for varying positions...
startpos = 1;
endpos = floor(length(signal)-framesize);

startpos2 = 1;
endpos2 = floor((length(signal2)-framesize)/3); %divided by 3 so can be repeated within outputLength

grainSpace = 5000; % needs to be implemented when frames are being reconstructed
grainSpace2 = 500; % should not be too large or will exceed array bounds..

numframes = floor(((endpos-startpos - framesize + hopsize) / hopsize)); %total samples - frame
numframes2 = floor((endpos2-startpos2 - framesize + hopsize) / hopsize);

outputLengthInput = 400000;
outputLength = ceil(outputLengthInput);

%divide spray level by 2 so that can take from both sides of grain
sprayFlag = 1; %spray enabled 1 disabled 0
sprayLevel = 0.5 / 2; % 1 < x < 100

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

framematrix = zeros(framesize,outputLength);
framematrix2 = zeros(framesize,outputLength);

% frame signals
framematrix = frame_signal(signal, startpos, hopsize, endpos, framesize);
framematrix2 = frame_signal(signal2, startpos2, hopsize, endpos2, framesize);

%% Reconstructing the signal
newsignal = zeros(startpos, endpos+framesize);
newsignal2 = zeros(startpos2, endpos2+framesize);

% reconstruct frames
newsignal = randomise_frames(newsignal, framematrix, numframes, hopsize, framesize);
newsignal2 = rebuild_frames(newsignal2, framematrix2, numframes2, hopsize, framesize);

%remove trailing zeros from newsignals
newsignal = newsignal(newsignal~=0); 
newsignal2 = newsignal2(newsignal2~=0);

%add trailing zeros equal to grainspacing
newsignal = [newsignal zeros(1, grainSpace)];
newsignal2 = [newsignal2 zeros(1, grainSpace2)];

figure; plot(newsignal);
figure; plot(newsignal2);

% needs to be 'generate signal' -> continually run attenuation on signal
% until length of outputLength
signalOutput = generateSignal(newsignal, outputLength, sprayFlag, sprayLevel);
signalOutput2 = generateSignal(newsignal2, outputLength, sprayFlag, sprayLevel);

figure;
subplot(211);
plot(signal);
title('Original');

subplot(212);
plot(signalOutput);
title('Random');

player = audioplayer(signalOutput,fs);
playblocking(player);

figure;
subplot(211);
plot(signal2);
title('Original');

subplot(212);
plot(signalOutput2);
title('Random');

player = audioplayer(signalOutput2,fs);
playblocking(player);


%add two signals together...
%{
for i = 1:length(newsignal)
    newsignal2(:,i) = (newsignal2(:,i) + newsignal(:,i)) / 2;
end
%}

%wavwrite(newsignal, fs, 'trumpted_recombined_randomized.wav');

for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)'; %starts from last
end


%this function is used to remove whitespace - zero values, and values inside of
%threshhold absolute value
function [out_signal] = remove_low_vals(signal)
    out_signal = signal( signal(:,1) < -0.0002 | signal(:,1) > 0.0002, :);
end

%this function breaks the signal into frames
function [framematrix] = frame_signal(signal, startpos, hopsize, endpos, framesize)
    %Decompose signal into frames
    for currentframe = startpos:hopsize:endpos %each frame is one [hopsize] from 1 to last sample-1frame
            thisframe = signal(currentframe:currentframe+framesize-1,1); %frame = the signal from 1 to framesize-1 then framesize to 2framesize-1....
            %thisframe = thisframe';
            framematrix(:,round(currentframe/hopsize)+1) = thisframe .* hanning(framesize); %this part was changed%%%
            %changed thisframe to (:,1) as each side was conflicting sizes
            %plot(thisframe' .* hanning(framesize));
            %pause;
    end
end

% rebuild frames into a signal (newsignal)
function [newsignal] = rebuild_frames(newsignal, framematrix, numframes, hopsize, framesize)
    % newsignal --- signal of length numframes
    % signalindex --- index of frames from 1 to framesize, framesize+1
    % to 2xframesize
    % newsignalRepeat --- signal of length outputLength
    for currentframe = 1:numframes
            signalindex = (currentframe-1) * hopsize + 1 : (currentframe-1) * hopsize + framesize;
            newsignal(signalindex) = newsignal(signalindex) + framematrix( :, (numframes - currentframe) +1 )';
    end % mod used to repeat signal...
end

% rebuild frames into a signal in a random order using randperm()
function [newsignal] = randomise_frames(newsignal, framematrix, numframes, hopsize, framesize)
    randframes = randperm(numframes); % create random numbers which sequece from first one (1) to number of frames: numframes
    for currentframe = 1:numframes
            signalindex = (currentframe-1) * hopsize + 1 : (currentframe-1) * hopsize + framesize;

            %newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)';
            newsignal(signalindex) = newsignal(signalindex) + framematrix( :, randframes(currentframe) )';
    end % mod used to repeat signal...
end

%repeat the signals until outputLength is reached
function [outputSignal] = generateSignal(newsignal, outputLength, sprayFlag, sprayLevel)
    %newsignalRepeat = repmat(newsignal,1,ceil(outputLength/length(newsignal)));
    %newsignalRepeat = newsignalRepeat(:,1:outputLength);
    
    signalIndex = 1;
    while (signalIndex < outputLength)
       if sprayFlag == 1
           tempSignal = attenuateSpray(tempSignal, sprayLevel)
       else
           tempSignal = newsignal; 
       end
       if (signalIndex + length(tempSignal)) > outputLength
           outputSignal = zeros(1, length(tempSignal));
           outputSignal(signalIndex) = outputSignal(signalIndex) + tempSignal(1, signalIndex : outputLength);
           signalIndex = outputLength;
       else
           outputSignal = zeros(1, length(tempSignal));
           outputSignal(signalIndex) = [outputSignal(signalIndex) tempSignal(length(tempSignal))];
           signalIndex = signalIndex + length(tempSignal);
       end
    end
end

%function spray
function [spraySignal] = attenuateSpray(newsignal, sprayLevel)
    a = rand * sprayLevel;
    b = rand * sprayLevel;
    %this variable is returning nothing...
    spraySignal = newsignal(1, 1 + floor((a * length(newsignal))) : floor(length(newsignal) - (b * length(newsignal))));
end