% Especificar la ruta del archivo .json
filename = 'D:\Users\Andres Yesid Romero\Downloads\Zohan S23.5dp9blvg.json'; 

% Leer el contenido del archivo como texto
jsonText = fileread(filename);

% Decodificar el texto JSON a una estructura de MATLAB
data = jsondecode(jsonText);

% Explorar sensores
sensors = data.payload.sensors;

% Ver los campos de un sensor (ejemplo: el primer sensor)
disp(fieldnames(sensors(1)));

% Mostrar información de todos los sensores
for i = 1:numel(sensors)
    disp(['Sensor ', num2str(i), ':']);
    disp(sensors(i)); % Muestra la información del sensor i
end

% Acceder a los valores
values = data.payload.values;

% Mostrar el tamaño de la matriz
disp(size(values)); 

% - sensors: contiene los nombres de los sensores
% - values: la matriz

tamano = size(values);

% Crear un vector X para el eje X (1 a filas)
x = 0:0.016:0.016*(tamano(1,1)-1);

% Iterar sobre los sensores y guardar cada columna de 'values' en un vector separado
for i = 1:numel(sensors)-3
    % Obtener el nombre del sensor
    sensorName = sensors(i).name;
    
    % Extraer la columna correspondiente de la matriz 'values'
    columnData = values(:, i);
    
    % Crear dinámicamente una variable con el nombre del sensor
    % Nota: eval crea variables con nombres dinámicos
    eval([sensorName, ' = columnData;']);
    
    % Graficar los datos
    figure; % Crear una nueva ventana de figura para cada gráfico
    plot(x, columnData, 'LineWidth', 1.5);
    title(['Gráfico de ', sensorName]);
    xlabel('segundos');
    ylabel([sensorName, ' (', sensors(i).units, ')']); % Mostrar la unidad del sensor
    grid on;
end

% Parámetros básicos
fs = 62.6;                  % Frecuencia de muestreo en Hz
t = linspace(0, 240, 15000); % Tiempo (240 segundos, 62.6 Hz)
n = length(t);              % Número de muestras
f = (0:n/2-1) * (fs / n);   % Vector de frecuencias (hasta Nyquist)

% Calcular la FFT de cada señal
fft_accel_x = fft(accX);
fft_accel_y = fft(accY);
fft_accel_z = fft(accZ);
fft_gyro_x = fft(gyroX);
fft_gyro_y = fft(gyroY);
fft_gyro_z = fft(gyroZ);

% Calcular el espectro de amplitud (magnitude)
amp_accX = abs(fft_accel_x) / n;
amp_accY = abs(fft_accel_y) / n;
amp_accZ = abs(fft_accel_z) / n;
amp_gyroX = abs(fft_gyro_x) / n;
amp_gyroY = abs(fft_gyro_y) / n;
amp_gyroZ = abs(fft_gyro_z) / n;

% Graficar el espectro de frecuencia (solo la mitad positiva)
% Espectro de frecuencia - Aceleración X
figure;
plot(f, 2 * amp_accX(1:n/2), 'r');  % Multiplicar por 2 para ajustar amplitud
title('Espectro de Frecuencia - Aceleración X');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Espectro de frecuencia - Aceleración Y
figure;
plot(f, 2 * amp_accY(1:n/2), 'g');
title('Espectro de Frecuencia - Aceleración Y');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Espectro de frecuencia - Aceleración Z
figure;
plot(f, 2 * amp_accZ(1:n/2), 'b');
title('Espectro de Frecuencia - Aceleración Z');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Espectro de frecuencia - Velocidad angular X
figure;
plot(f, 2 * amp_gyroX(1:n/2), 'b');
title('Espectro de Frecuencia - Velocidad angular X');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Espectro de frecuencia - Velocidad angular Y
figure;
plot(f, 2 * amp_gyroY(1:n/2), 'b');
title('Espectro de Frecuencia - Velocidad angular Y');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Espectro de frecuencia - Velocidad angular Z
figure;
plot(f, 2 * amp_gyroZ(1:n/2), 'b');
title('Espectro de Frecuencia - Velocidad angular Z');
xlabel('Frecuencia (Hz)');
ylabel('Amplitud');
grid on;

% Parámetros del filtro
fs = 62.6;            % Frecuencia de muestreo en Hz
nyquist = fs / 2;     % Frecuencia de Nyquist
cutoff = 1;          % Frecuencia de corte del filtro en Hz
order = 4;            % Orden del filtro

% Diseñar el filtro Butterworth pasa-bajos
[b, a] = butter(order, cutoff / nyquist, 'low');

% Aplicar el filtro a los datos
filtered_accel_x = filtfilt(b, a, accX);
filtered_accel_y = filtfilt(b, a, accY);
filtered_accel_z = filtfilt(b, a, accZ);
filtered_gyro_x = filtfilt(b, a, gyroX);
filtered_gyro_y = filtfilt(b, a, gyroY);
filtered_gyro_z = filtfilt(b, a, gyroZ);

% Graficar los resultados
% Aceleración X (comparacion)
figure;
plot(t, accX, 'r', 'DisplayName', 'Original accX'); hold on;
plot(t, filtered_accel_x, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accX');
legend();
title('Aceleración X');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Y (comparacion)
figure;
plot(t, accY, 'r', 'DisplayName', 'Original accY'); hold on;
plot(t, filtered_accel_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accY');
legend();
title('Aceleración Y');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Z (comparacion)
figure;
plot(t, accZ, 'r', 'DisplayName', 'Original accZ'); hold on;
plot(t, filtered_accel_z, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accZ');
legend();
title('Aceleración Z');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Velocidad angular X (comparacion)
figure;
plot(t, gyroX, 'r', 'DisplayName', 'Original gyroX'); hold on;
plot(t, filtered_gyro_x, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado gyroX');
legend();
title('Velocidad angular X');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (grados/s)');
grid on;

% Velocidad angular Y (comparacion)
figure;
plot(t, gyroY, 'r', 'DisplayName', 'Original gyroY'); hold on;
plot(t, filtered_gyro_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado gyroY');
legend();
title('Velocidad angular Y');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (grados/s)');
grid on;

% Velocidad angular Z (comparacion)
figure;
plot(t, gyroZ, 'r', 'DisplayName', 'Original gyroZ'); hold on;
plot(t, filtered_gyro_z, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado gyroZ');
legend();
title('Velocidad angular Z');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (grados/s)');
grid on;

% Aceleración X (filtrada)
figure;
plot(t, filtered_accel_x, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accX');
legend();
title('Aceleración X');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Y (filtrada)
figure;
plot(t, filtered_accel_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accY');
legend();
title('Aceleración Y');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Z (filtrada)
figure;
plot(t, filtered_accel_z, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accZ');
legend();
title('Aceleración Z');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

t2 = t(1,1:10000);

% Aceleración X (recortada y contaminada)
filtered_accel_x = filtered_accel_x(501:10500,1);
figure;
plot(t2, filtered_accel_x, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accX');
legend();
title('Aceleración X');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Z (recortada y contaminada)
filtered_accel_z = filtered_accel_z(501:10500,1);
figure;
plot(t2, filtered_accel_z, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accZ');
legend();
title('Aceleración Z');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Velocidad X (contaminada)
velX = cumtrapz(filtered_accel_x);
figure;
plot(t2, velX, 'b');
title('Velocidad X');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
grid on;

% Velocidad Z (contaminada)
velZ = cumtrapz(filtered_accel_z);
figure;
plot(t2, velZ, 'b');
title('Velocidad Z');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
grid on;

% Aceleración X (descontaminada)
filtered_accel_x = filtered_accel_x + 4.1;
figure;
plot(t2, filtered_accel_x, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accX');
legend();
title('Aceleración X');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Aceleración Z (descontaminada)
filtered_accel_z = filtered_accel_z + 1.2;
figure;
plot(t2, filtered_accel_z, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado accZ');
legend();
title('Aceleración Z');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s^2)');
grid on;

% Velocidad X (descontaminada)
velX = cumtrapz(filtered_accel_x);
figure;
plot(t2, velX, 'b');
title('Velocidad X');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
grid on;

% Velocidad Z (descontaminada)
velZ = cumtrapz(filtered_accel_z);
figure;
plot(t2, velZ, 'b');
title('Velocidad Z');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
grid on;

% Velocidad angular Y
filtered_gyro_y = filtered_gyro_y(501:10500,1);
figure;
plot(t2, filtered_gyro_y, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtrado gyroY');
legend();
title('Velocidad angular Y');
xlabel('Tiempo (s)');
ylabel('Velocidad angular (grados/s)');
grid on;

% Velocidad Y (descontaminada)
despAngY = cumtrapz(filtered_gyro_y);
figure;
plot(t2, despAngY, 'b');
title('Desplazamiento angular Y');
xlabel('Tiempo (s)');
ylabel('grados');
grid on;

% Desplazamiento Z
despZ = cumtrapz(velZ);
figure;
plot(t2, despZ, 'b');
title('Desplazamiento Z');
xlabel('Tiempo (s)');
ylabel('m');
grid on;

%Trayectoria estimada
deltaZ = diff(despZ);
despAngY = despAngY(1:length(despAngY)-1,1);
componentes_X = deltaZ.*cosd(despAngY);
componentes_Y = deltaZ.*sind(despAngY);
X = cumsum(componentes_X);
Y = cumsum(componentes_Y);
plot(X,Y);
title('Trayectoria estimada');
xlabel('X (m)');
ylabel('Y (m)');