%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Granular Synthesis - Morphing Script
% UTS Capstone Project 2018
% Student: James Dolphin
% Supervisor: Dr. Eva Cheng
% Base dervied from MMI 502 Lab 7 - Seth Hochberg
% https://github.com/sethhochberg/matlab_granular_synthesis/blob/master/Lab7.m
% program uses old wavread function which requires an include, and the
% Matlab Communications System Toolbox must be installed for FM Modulation

%Could show Mark's GUI

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

%%%%%% need to normalise the signals:  between -1 and 1
% amplitude normalisation - divide by the maximum
signal = ampNormalise(signal);
signal2 = ampNormalise(signal2);

%should be decided based on max frequency / amplitude of message signal?
figure; plot(signal); hold on; plot(attenuateRealAM(signal, 10000, fs),'r');

%need to trim zeros from beginning + end when reading signals AND adjust
signal = remove_low_vals(signal); % add threshhold value
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

outputLengthInput = 400000; %output length of granular synthesis
outputLength = ceil(outputLengthInput);

%divide spray level by 2 so that can take from both sides of grain
sprayFlag1 = 0; %spray enabled 1 disabled 0
sprayLevel1 = 0.5; % 0 (no spray) < sprayLevel1 < 0.5 (full spray)
sprayLoops1 = 10;

sprayFlag2 = 0;
sprayLevel2 = 0.5;
sprayLoops2 = 10;

%this level is multiplied by a rand value, could also be skewed to increase
%spray

AMFlag1 = 0;
AMLevel1 = 50 / 100; % 50% Amplitude Modulation - this means that the signal can be modulated to 0
AMLoops1 = 10;

AMFlag2 = 0;
AMLevel2 = 50 / 100; % 50% Amplitude Modulation
AMLoops2 = 10;

realAMFlag1 = 1;
realAMFc1 = 10000; % carrier frequency
realAMFlag2 = 1;
realAMFc2 = 10000;

FMFlag1 = 0;
FMLevel1 = 50 / 100; % 50% Frequency Modulation
FMFlag2 = 0;
FMLevel2 = 50 / 100; % 50% Frequency Modulation

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

% needs to be 'generate signal' -> continually run attenuation on signal
% until length of outputLength
signalOutput = generateSignal(newsignal, fs, grainSpace, outputLength, sprayFlag1, sprayLevel1, sprayLoops1, AMFlag1, AMLevel1, AMLoops1, FMFlag1, realAMFlag1, realAMFc1);
signalOutput2 = generateSignal(newsignal2, fs, grainSpace2, outputLength, sprayFlag2, sprayLevel2, sprayLoops2, AMFlag2, AMLevel2, AMLoops2, FMFlag2, realAMFlag2, realAMFc2);

%parametricMorphing(signalOutput, signalOutput2, sprayFlag1, sprayFlag2,
%sprayLevel2, sprayLevel2, AMFlag1, AMFlag2, AMLevel1, AMLevel2);

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

outputSignal = morphSignals(signalOutput, signalOutput2, outputLength);

player = audioplayer(outputSignal, fs);
playblocking(player);

figure; plot(outputSignal);

%wavwrite(newsignal, fs, 'trumpted_recombined_randomized.wav');

for currentframe = 1:numframes
        signalindex = (currentframe-1)*hopsize+1:(currentframe-1)*hopsize+framesize;
        newsignal(signalindex) = newsignal(signalindex) + framematrix(:,(numframes-currentframe)+1)'; %starts from last
end

%this function is used to remove whitespace - zero values, and values inside of
%threshhold absolute value
function [out_signal] = remove_low_vals(signal)
    out_signal = signal( signal(:,1) < -0.0002 | signal(:,1) > 0.0002, :);
    %find cofrect values and []
end

function [out_signal] = ampNormalise(tempSignal)
    %find maxval in signal
    %out_signal = zeros(length(tempSignal(:,1)), 1);
    
    maxVal = max(tempSignal(:,1))
    %for i = 1:length(tempSignal)
    %   if maxVal < tempSignal(1,:)
    %        maxVal = tempSignal(1,:);
    %   end
    % end
   
    out_signal(:,1) = tempSignal(:,1) / maxVal;
    
    figure; plot(tempSignal);
    hold on; plot(out_signal, 'r');
    
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

%repeat the grain and apply attenuation throughout until outputLength is reached

function [outputSignal] = generateSignal(newsignal, Fs, grainSpace, outputLength, sprayFlag, sprayLevel, sprayLoops, AMFlag, AMLevel, AMLoops, FMFlag, realAMFlag, realAMFc)

    i = 1;
    outputSignal = zeros(1, outputLength);
    loopCount = 1;
    loopNumber = 10;
    
    %each granular iteration - signal is changing signal until morphing is
    %achieved
    while(i <= outputLength)
       tempSignal = newsignal;
       % 
       %%parameters change based on loopCount - can check against
       %%nonchanging parameters to demonstrate parametric morphing
       %%%% apply attenuation parameters
       if sprayFlag == 1 && loopCount <= sprayLoops
           for j = 1:loopNumber
               %sprayLevel needs to vary - start Value - end Value
               %check sprayLevel change and 
               
               tempSignal = attenuateSpray(tempSignal, loopCount/10 * sprayLevel);
               
           end
       end
       
       if AMFlag == 1 && loopCount <= AMLoops
           tempSignal = attenuateAM(tempSignal, AMLevel);
       end
       
       if FMFlag == 1
           tempSignal = attenuateFM(tempSignal);
       end
       
       if realAMFlag == 1
           tempSignal = attenuateRealAM(tempSignal,realAMFc, Fs);
       end
       
        %add trailing zeros equal to grainspacing
        tempSignal = [tempSignal zeros(1, grainSpace)];
        
        %figure; plot(tempSignal);
        
       for j = 1:length(tempSignal)
          outputSignal(1, i) = tempSignal(1, j);
          i = i + 1;
          if (i >= outputLength + 1)
              break;
          end
       end
       
       loopCount = loopCount + 1;
       
    end
end

%function spray
function [spraySignal] = attenuateSpray(tempSignal, sprayLevel1)
    a = rand;
    b = rand;
    while(a > sprayLevel1)
        a = rand;
    end
    while(b > sprayLevel1)
        b = rand;
    end
    %this variable is returning nothing...
    spraySignal = tempSignal(1, 1 + floor((a * length(tempSignal))) : floor(length(tempSignal) - (b * length(tempSignal))));
end

%Amplitude Randomisation - the amplitude of grains is randomly varied based on
%AMLevel

function [AMSignal] = attenuateAM(tempSignal, AMLevel)
    a = rand;
    while(a > AMLevel) % a must be less than critical value
        a = rand;
    end
        
    for i = 1:length(tempSignal)
        tempSignal(1, i) = tempSignal(1, i) * a;
    end
    
    AMSignal = tempSignal;
end

function [AMSignal2] = attenuateRealAM(tempSignal, Fc, Fs)
    % Carrier signal (Sc) = Acsin(2?fct)
    % Message signal (Sm) = Amsin(2?fmt)
    % m=Am/Ac
    % Modulated Signal = (Ac+ Amsin(2 ?fmt))*sin(2 ?fct)
    
    % Fs = 8000;                        % Sampling rate is 8000 samples per second.
    % Fc = 300;                         % Carrier frequency in Hz
    % t = [0:.1*Fs]'/Fs;                % Sampling times for .1 second
    % x = sin(20*pi*t);                 % Representation of the signal
    % y = ammod(x,Fc,Fs);               % Modulate x to produce y.
    %
    AMSignal2 = ammod(tempSignal,Fc,Fs);
end

%FM Synthesis
function [FMSignal] = attenuateFM(tempSignal)   
    Fc = 1e3;         % carrier = 1MHz - typical length is 100MHz/3m?
    FS = 2.2*Fc;      % sampling frequency for output signal
    deviation = 1e2; % freq. deviation
    FMSignal = fmmod(tempSignal, Fc, FS, deviation); %matlab frequency modulation function
end

%add AM
%morph the two signals using simple summation
function [outputSignal] = morphSignals(signal1, signal2, outputLength)
    outputSignal = zeros(1, outputLength);
    %add two signals to outputSignal, and divide by 2..
    for i = 1:length(signal1)
        outputSignal(:, i) = outputSignal(:, i) + signal1(:, i);
    end
    
    for i = 1:length(signal2)
        outputSignal(:, i) = outputSignal(:, i) + signal2(:, i);
    end
    
    for i = 1:outputLength
        outputSignal(:, i) = outputSignal(:, i) / 2;
    end
end