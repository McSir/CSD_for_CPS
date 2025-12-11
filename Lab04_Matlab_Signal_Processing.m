clc, clearvars, close all

%% Lab04: Generation, Processing, and Analysis of Sine Waves

%% 1) Read Data
% Load the provided signal 'signal.csv' using the 'csvread()' function and
% split the signal and time into two separate vectors.

fprintf('1) Start.\n')

opts = detectImportOptions('signal.csv');
opts.SelectedVariableNames = opts.VariableNames([4 5]);  % keep only numeric cols 4 & 5
T = readtable('signal.csv', opts);

time   = T{:,1};
signal = T{:,2};

figure(1)
plot(time, signal);
xlabel('Time [s]');
ylabel('Amplitude');
title('Loaded Signal');
grid on;

fprintf('1) Done.\n')

%% 2/3) Butterworth Filter
% Design a Butterworth filter that fulfills the required characteristics 
% given in the assignment description. Use the built-in functions of
% Matlab. The 'doc' and 'help' functions might be useful to obtain detailed
% information.

% 2) First, calculate the required filter order and cutoff frequency and
% print the result.

fprintf('2) Start.\n')
% --- Infer sampling frequency from time vector ---
dt = median(diff(time(~isnan(time))));   % robust step estimate
Fs = 1/dt;                               % sampling frequency [Hz]

% === REQUIRED SPECS (SET THESE PER ASSIGNMENT) ===
Fp  = 100;    % passband edge frequency [Hz]      <-- TODO: set
Fst = 300;    % stopband edge frequency [Hz]      <-- TODO: set
Rp  = 1;      % passband ripple [dB]              <-- TODO: set
Rs  = 40;     % stopband attenuation [dB]         <-- TODO: set

% --- Normalize to Nyquist ---
Wp = Fp  / (Fs/2);
Ws = Fst / (Fs/2);

% --- Compute Butterworth order & cutoff ---
[n, Wn] = buttord(Wp, Ws, Rp, Rs);

% Cutoff in Hz for reporting (scalar for lowpass/highpass)
fc_Hz = Wn * (Fs/2);

fprintf('Sampling rate Fs = %.3f Hz\n', Fs);
fprintf('Butterworth order n = %d\n', n);
fprintf('Cutoff (normalized) Wn = %.6f\n', Wn);
fprintf('Cutoff frequency fc = %.3f Hz\n', fc_Hz);

fprintf('2) Done.\n')

% 3) Calculate the filter coefficients and apply them to the signal, i.e.,
% filter the signal. Plot the filtered signal into the same figure as the
% original signal. Make sure to add a title, the axis descriptions, and a
% legend.

fprintf('3) Start.\n')

% --- Design lowpass Butterworth and filter ---
[b, a] = butter(n, Wn, 'low');        % 'low' for lowpass; change if needed
y = filtfilt(b, a, signal);           % zero-phase filtering

% --- Plot on the same figure as original ---
figure(1); hold on;
plot(time, y, 'LineWidth', 1.2);
title('Original vs. Butterworth-Filtered Signal');
xlabel('Time [s]'); ylabel('Amplitude');
legend('Original', sprintf('Butterworth LP (n=%d, fc=%.0f Hz)', n, fc_Hz), 'Location', 'best');
grid on; hold off;


fprintf('3) Done.\n')

%% 4. Fourier Transform
% Calculate the single-sided Fourier transform of the filtered signal.

fprintf('4) Start.\n')

% 4.1) First, obtain the length of the original and filtered signal and 
% calculate their means. Print both mean values.

% Ensure column vectors and remove any NaNs (safety)
t = time(:);
x = signal(:);
yf = y(:);

valid_x  = ~isnan(x);
valid_yf = ~isnan(yf);

x  = x(valid_x);
yf = yf(valid_yf);

Lx  = numel(x);
Lyf = numel(yf);

mx  = mean(x,  'omitnan');
myf = mean(yf, 'omitnan');

fprintf('Original: length = %d, mean = %.6g\n', Lx,  mx);
fprintf('Filtered: length = %d, mean = %.6g\n', Lyf, myf);

% 4.2) Do the FFT for both signals; remove the mean first.

% 4.2.1) Original signal
x0 = x - mx;                      % remove DC
X  = fft(x0);
P2x = abs(X / Lx);                % two-sided amplitude spectrum
P1x = P2x(1:floor(Lx/2)+1);       % single-sided
if numel(P1x) > 2
    P1x(2:end-1) = 2*P1x(2:end-1);
end
fx = Fs*(0:floor(Lx/2))/Lx;       % frequency axis (Hz)

% 4.2.2) Filtered signal
y0 = yf - myf;                    % remove DC
Y  = fft(y0);
P2y = abs(Y / Lyf);
P1y = P2y(1:floor(Lyf/2)+1);
if numel(P1y) > 2
    P1y(2:end-1) = 2*P1y(2:end-1);
end
fy = Fs*(0:floor(Lyf/2))/Lyf;

% 4.2.3) When plotting, only visualize the spectrum up to 500 Hz.
fmax = 500;                       % Hz
ix = fx <= fmax;
iy = fy <= fmax;

figure(2)
plot(fx(ix), P1x(ix), 'DisplayName', 'Original'); hold on;
plot(fy(iy), P1y(iy), 'DisplayName', 'Filtered');
xlabel('Frequency [Hz]');
ylabel('Amplitude');
title('Single-Sided Amplitude Spectrum');
legend('show', 'Location', 'best');
grid on; hold off;

fprintf('4) Done.\n')

%% 5. Frequency Identification
% Write a function that automatically detects a signals frequency based
% on its frequency spectrum. You can assume there's only a single signal
% and noise has been removed. The function must return the amplitude and
% the frequency of this signal.

fprintf('5) Start.\n')

% 5.2) What is the frequency of the signal you have analyzed?
% (Recompute the single-sided spectrum for the filtered signal y)

yf = y(:);                                   % filtered signal from section 3
L  = numel(yf);
y0 = yf - mean(yf, 'omitnan');

Y   = fft(y0) / L;                           % normalize by length
P2  = abs(Y);                                % two-sided amplitude
P1  = P2(1:floor(L/2)+1);                    % single-sided amplitude
if numel(P1) > 2
    P1(2:end-1) = 2*P1(2:end-1);            % standard single-sided scaling
end
f   = Fs * (0:floor(L/2)) / L;               % frequency axis (Hz)

[amp_detected, f0_detected] = detect_tone(f, P1);

fprintf('Detected frequency f0 = %.3f Hz, amplitude = %.6g\n', f0_detected, amp_detected);

fprintf('5) Done.\n')

% 5.1) Define function
function [A, f0] = detect_tone(f, P1)
%DETECT_TONE  Detects dominant tone from single-sided spectrum.
%   Inputs:
%     f  - frequency vector (Hz), length N
%     P1 - single-sided amplitude spectrum (matching f), length N
%   Outputs:
%     A  - estimated amplitude of the tone (matches time-domain amplitude
%          for a pure sinusoid with the Section 4 scaling)
%     f0 - estimated frequency (Hz)
%
%   Notes:
%   - DC (bin 1) is ignored.
%   - Parabolic interpolation around the peak refines the estimate.

    N = numel(P1);
    if N < 2
        % Degenerate case: no bins to search
        [A, idx] = max(P1);
        f0 = f(idx);
        return;
    end

    % Ignore DC bin when searching
    [~, kRel] = max(P1(2:end));
    k = kRel + 1;  % convert to absolute index

    % Parabolic interpolation (optional but improves accuracy):
    if k > 1 && k < N
        alpha = P1(k-1);
        beta  = P1(k);
        gamma = P1(k+1);

        % Vertex offset p in bins (see Quinn or standard parabolic peak fit)
        denom = (alpha - 2*beta + gamma);
        if denom ~= 0
            p = 0.5 * (alpha - gamma) / denom;    % between -1 and 1 ideally
        else
            p = 0;
        end

        df  = f(2) - f(1);                        % frequency bin width
        f0  = f(k) + p * df;                       % refined frequency
        A   = beta - 0.25 * (alpha - gamma) * p;   % refined peak height
    else
        % Edge bin: no interpolation
        f0 = f(k);
        A  = P1(k);
    end
end
 