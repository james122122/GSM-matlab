function varargout = Granulator_GUI(varargin)
% GRANULATOR_GUI MATLAB code for Granulator_GUI.fig
%      GRANULATOR_GUI, by itself, creates a new GRANULATOR_GUI or raises the existing
%      singleton*.
%
%      H = GRANULATOR_GUI returns the handle to a new GRANULATOR_GUI or the handle to
%      the existing singleton*.
%
%      GRANULATOR_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRANULATOR_GUI.M with the given input arguments.
%
%      GRANULATOR_GUI('Property','Value',...) creates a new GRANULATOR_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Granulator_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Granulator_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Granulator_GUI

% Last Modified by GUIDE v2.5 12-Oct-2018 17:27:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Granulator_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Granulator_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

end
% End initialization code - DO NOT EDIT


% --- Executes just before Granulator_GUI is made visible.
function Granulator_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to Granulator_GUI (see VARARGIN)

    % Choose default command line output for Granulator_GUI
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes Granulator_GUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = Granulator_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
end

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2

end
% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
    str = get(hObject, 'String');
    val = get(hObject, 'Value');
    switch str(val)
        case 'Violin'
            handles.sample = 'violin-a4.wav';
        case 'Trumpet' 
            handles.sample = 'trumpet.wav';
    end
    
    [handles.signal handles.fs bitdepth] = wavread(handles.sample);
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    %[handles.signal handles.fs bitdepth] = wavread(handles.sample); % [sample data - sample rate - number of bits per sample]
    handles = get(hObject);
    
    
    %%%%%% need to normalise the signals:  between -1 and 1
    % amplitude normalisation - divide by the maximum
    handles.signal = ampNormalise(handles.signal);

    %should be decided based on max frequency / amplitude of message signal?
    %figure; plot(signal); hold on; plot(attenuateRealAM(signal, 11000, fs),'r');

    %need to trim zeros from beginning + end when reading signals AND adjust
    handles.signal = remove_low_vals(handles.signal); % add threshhold value

    handles.player = audioplayer(handles.signal,handles.fs);
    [handles.signal handles.fs bitdepth] = wavread(handles.sample);
    
    handles.player = audioplayer(handles.signal, handles.fs);
    
    playblocking(handles.player);
    playblocking(handles.player);

end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

end

% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end


% --- Executes on slider movement.
function slider12_Callback(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
end

% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
end

% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
end

% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3
end

% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4
end

% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5
end

% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
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
       
       if AMFlag == 0 && loopCount <= AMLoops
           tempSignal = attenuateAM(tempSignal, AMLevel);
       end
       
       if FMFlag == 0
           tempSignal = attenuateFM(tempSignal, Fs);
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
    AMSignal2 = ammod(tempSignal, Fc, Fs);
%    AMSignal2 = amdemod(AMSignal2, Fc, Fs);
end

%FM Synthesis
function [FMSignal] = attenuateFM(tempSignal, Fs)   
   % Fs = 2.2 * Fc;      % sampling frequency for output signal 
    % Fc = Fs / 2.2; % carrier ~ 44000 / 2.2
    Fc = 15 * 1000;
    deviation = 25 * 1000; % freq. deviation 2kHz % must always be greater than the carrier~~
    % 10 kHz deviation is equivalent to a 5 KBPS change in bit rate
    FMSignal = fmmod(tempSignal, Fc, Fs, deviation); %matlab frequency modulation function
    FMSignal = fmdemod(FMSignal, Fc, Fs, deviation);
    %m=Frequency deviationModulation frequency
    %Upper modulating frequency: 15 kHz
    
    % max frequency should be half sampling frequency : 44100 / 2 = 22050
    % CBR=2(\Delta f+f_{m}) - Carson's rule: where Delta f is frequency
    % deviation, and f_{m} is the max frequency
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
