clear;
close all;
clc;
addpath('Quaternions');
addpath('Podaci sa senzora');
% -------------------------------------------------------------------------
filePath = 'data.csv';
data = readtable(filePath, 'VariableNamingRule', 'preserve');
startTime = 0; % Postavi vreme početka analize
stopTime = 2; % Postavi vreme kraja analize
% -------------------------------------------------------------------------
% Import data
samplePeriod = 1/256; % Odredi period uzorkovanja
numSamples = height(data); % Odredi broj uzoraka u podacima
time = (0:numSamples-1)' * samplePeriod; % Kreiraj vreme
% -------------------------------------------------------------------------
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1); % Odredi indekse uzoraka unutar vremenskog opsega
time = time(indexSel); % Izaberi vremenske podatke na osnovu indeksa
gyrX = data.("Gyroscope X (deg/s)");
gyrY = data.("Gyroscope Y (deg/s)");
gyrZ = data.("Gyroscope Z (deg/s)");
accX = data.("Accelerometer X (g)");
accY = data.("Accelerometer Y (g)");
accZ = data.("Accelerometer Z (g)");
time = time(indexSel);
gyrX = gyrX(indexSel);
gyrY = gyrY(indexSel);
gyrZ = gyrZ(indexSel);
accX = accX(indexSel);
accY = accY(indexSel);
accZ = accZ(indexSel);
% -------------------------------------------------------------------------
% Otkrivanje stacionarnih perioda

% Izračunavanje veličine akcelerometra
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ); % Izračunaj magnitudu ubrzanja

% HP filter accelerometer data
filtCutOff = 0.5; % Postavi frekvencijski prag za visoko-propusni filtar
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high'); % Kreiraj koeficijente filtra
acc_magFilt = filtfilt(b, a, acc_mag); % Primeni visoko-propusni filtar na magnitudu ubrzanja

% Compute absolute value
acc_magFilt = abs(acc_magFilt); % Izračunaj apsolutnu vrednost filtriranog signala

% LP filter accelerometer data
filtCutOff = 6; % Postavi frekvencijski prag za nisko-propusni filtar
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low'); % Kreiraj koeficijente filtra
acc_magFilt = filtfilt(b, a, acc_magFilt); % Primeni nisko-propusni filtar na magnitudu ubrzanja

% Threshold detection
stationary = acc_magFilt < 0.09; % Detektuj stacionarne periode na osnovu praga

% -------------------------------------------------------------------------
% Create a single figure with subplots
figure('Position', [100, 100, 900, 900], 'NumberTitle', 'off', 'Name', 'Sensor Data and Motion Analysis'); % Kreiraj figuru sa podgrafikonima

% Subplot 1: Gyroscope data
subplot(4, 1, 1);
hold on;
plot(time, gyrX, 'r');
plot(time, gyrY, 'g');
plot(time, gyrZ, 'b');
title('Gyroscope');
xlabel('Time (s)');
ylabel('Angular velocity (^\circ/s)');
legend('X', 'Y', 'Z');
hold off;

% Subplot 2: Accelerometer data
subplot(4, 1, 2);
hold on;
plot(time, accX, 'r');
plot(time, accY, 'g');
plot(time, accZ, 'b');
plot(time, acc_magFilt, ':k');
plot(time, stationary, 'k', 'LineWidth', 2);
title('Accelerometer');
xlabel('Time (s)');
ylabel('Acceleration (g)');
legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
hold off;

% Compute orientation
quat = zeros(length(time), 4); % Inicijalizuj matricu za kvaternione
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1); % Kreiraj AHRS objekat

% Initial convergence
initPeriod = 2; % Postavi vreme za inicijalnu konvergenciju
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1); % Odredi indekse za inicijalnu konvergenciju
for i = 1:2000 % Petlja za inicijalizaciju
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]); % Ažuriraj AHRS algoritam
end

% For all data
for t = 1:length(time) % Petlja kroz sve vremenske uzorke
    if(stationary(t)) % Proveri da li je trenutni uzorak stacionaran
        AHRSalgorithm.Kp = 1; % Postavi Kp na 1 za stacionarne periode
    else
        AHRSalgorithm.Kp = 0; % Postavi Kp na 0 za nestacionarne periode
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]); % Ažuriraj AHRS algoritam sa novim podacima
    quat(t,:) = AHRSalgorithm.Quaternion; % Sačuvaj kvaternion
end

% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat)); % Rotiraj ubrzanje u Zemljin okvir

% Convert acceleration measurements to m/s/s
acc = acc * 9.81; % Pretvori ubrzanje u m/s²

subplot(4, 1, 3);
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end

% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% Subplot 4: Translational velocity
subplot(4, 1, 4);
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% Adjust the layout
sgtitle('Sensor Data and Motion Analysis');
linkaxes(findall(gcf,'type','axes'),'x'); % Link all x-axes for synchronized zooming

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81; % Korekcija akceleracije na Z osi za gravitaciju

% Integrate acceleration to yield velocity
vel = zeros(size(acc)); % Inicijalizuj matricu za brzinu
for t = 2:length(vel) % Petlja kroz sve uzorke
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod; % Ažuriraj brzinu integracijom akceleracije
    if(stationary(t) == 1) % Proveri da li je sensor stacionaran
        vel(t,:) = [0 0 0];     % nateraj brzinu na nulu kada je stopalo stacionarno
    end
end

% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel)); % Inicijalizuj matricu za drift brzine
stationaryStart = find([0; diff(stationary)] == -1); % Pronađi početke stacionarnih perioda
stationaryEnd = find([0; diff(stationary)] == 1); % Pronađi krajeve stacionarnih perioda
for i = 1:numel(stationaryEnd) % Petlja kroz sve završetke stacionarnih perioda
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i)); % Izračunaj brzinu drifta
    enum = 1:(stationaryEnd(i) - stationaryStart(i)); % Definiši niz za drift
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)]; % Izračunaj drift za svaki uzorak
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift; % Ažuriraj drift u matrici
end

% Remove integral drift
vel = vel - velDrift; % Ukloni drift iz brzine

subplot(4, 1, 4);
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% Adjust the layout
sgtitle('Sensor Data and Motion Analysis'); % Glavni naslov za sve podgrafike
linkaxes(findall(gcf,'type','axes'),'x'); % Poveži sve X ose za sinhronizovano zumiranje

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel)); % Inicijalizuj matricu za poziciju
for t = 2:length(pos) % Petlja kroz sve uzorke
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % Integracija brzine da se dobije pozicija
end

% -------------------------------------------------------------------------
% Plot 3D
posPlot = pos*5; % Skaliraj poziciju
quatPlot = quat; % Izvrni kvaternioni za animaciju
extraTime = 1; % Dodatno vreme za animaciju
onesVector = ones(extraTime*(1/samplePeriod), 1); % Vektor sa jedinicama za produženje
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]]; % Dodaj poslednji uzorak
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]]; % Dodaj poslednji kvaternion

% Kreiraj 6DOF animaciju
SamplePlotFreq = 5; % Frekvencija uzorkovanja za prikaz
Spin = 100; % Stepen rotacije u animaciji
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ... 
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq)); % Postavke animacije
