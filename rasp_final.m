function rasp_final()
    %% Implementación del algoritmo de estimación de la velocidad de un vehículo
    
    %{
    Paper de referencia:
    "Speed Estimation On Moving Vehicle Based On Digital Image Processing"
    (https://www.researchgate.net/publication/317312246)
    %}

    % Autores: Oscar Ortiz Torres, Yonathan Romero Amador y Emmanuel Lechuga Arreola

    %% 0. Definición de constantes, objetos y variables
    r = raspi('192.168.33.204', 'YRA', 'yra');  % Conexión con la Raspberry Pi
    %{
    Tv = 67.6; % Ángulo
    H = 2.12; % Altura de la cámara en m
    v = 0.050; %% Dimensión vertical del formato de imagen
    f = 0.002; % Distancia focal de la cámara
    %}
    foregroundDetector = vision.ForegroundDetector('NumGaussians', 10, ...
                                                   'NumTrainingFrames', 35, ...
                                                   'LearningRate', 0.002, ...
                                                   'MinimumBackgroundRatio', 0.7);
    
    prev_centroide = [0 0];
    vel_vector=[];
    flag = 0;
    
    cam = webcam(r); % Acceso a la cámara web
    fps = 29.73;
    %{
    % Calibración del campo de visión y factor de conversión
    Tc = 2 * atan(v / (2 * f)); % Campo de visión de la cámara
    T = Tv + Tc / 2; % Ángulo total de visión
    D = H * tan(T); % Distancia de la cámara al objeto
    I_height = cam.Resolution(2); % Altura del video en pixeles
    P = 2 * tan(Tc / 2) * sqrt(H^2 + D^2); % Proyección del campo de visión (en metros)
    k = P / I_height; % Factor de conversión de pixeles a metros
    %}
    I_height = cam.Resolution(2);
    k = 0.0317;
    %% Procesamiento en tiempo real
    for f = 1:120
        %% B. Preprocessing
        img = snapshot(cam);    % Captura del frame de la cámara
        img_pre = im2gray(img);
        img_pre = imadjust(img_pre, [0.32 0.92], [0 1], 2);
        
        %% C. Background Subs (restar el fondo del frame actual) & E. Shadow Removal
        bg_sub = step(foregroundDetector, img_pre);
    
        %% D. Smoothing
        img_suav = medfilt2(bg_sub, [20 20]);
    
        %% F. Operaciones morfológicas para eliminar ruido
        img_morfo = imopen(img_suav, strel('square', 4));
        img_morfo = imclose(img_morfo, strel('square', 15));
        
        %% G. Object detection
        img_morfo_uint8 = uint8(img_morfo) * 255;
        [labels, num] = bwlabel(img_morfo_uint8);
        stats = regionprops(labels, 'BoundingBox', 'Centroid', 'Area');
        
        %% H. Labeling and tracking
        %imshow(img)    % Se reemplaza por la línea 60, con la función displayImage, ya que imshow no es soportado por la Rasp
        for i = 1:num
            if stats(i).Area > 100 && flag == 0
                bbox = stats(i).BoundingBox;
                centroide = stats(i).Centroid;

                %%%
                % Coordenada inferior del objeto (bounding box)
                y_inferior = bbox(2) + bbox(4); % y_inicial + altura del bounding box
        
                % Condición para ignorar objetos que alcanzan el borde inferior del frame
                if y_inferior >= I_height - 10 % Margen de 10 píxeles
                    %continue; % Salta al siguiente objeto
                    flag = 1;
                end
                %%%

                %rectangle('Position', bbox, 'EdgeColor', 'y','LineWidth',%2); % Se reemplaza por la línea 57, con la función insertShape en lugar de rectangle
                img = insertShape(img, 'Rectangle', bbox, 'Color', [255, 255, 0], 'LineWidth', 2);
                %plot(centroide(1), centroide(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Se reemplaza por la linea 59 con la función insertShape en lugar de plot
                img = insertShape(img, 'Circle', [centroide(1), centroide(2), 5], 'Color', [255, 0, 0], 'LineWidth', 3);
                %displayImage(r, img, 'Title', 'Fotomulta');
                
                %% I. Speed Estimation
                d_e = sqrt(sum((prev_centroide - centroide) .^ 2));
                vel = 3.6 * (k * (d_e / (1 / fps)));
                vel_vector = [vel_vector, vel];
                
                % Guardado de coordenadas para la siguiente iteración
                prev_centroide = centroide;
            end
        end
    displayImage(r, img, 'Title', 'Fotomulta');
    end
    vel_vector(1) = []; % Eliminación de la primera muestra

    % Impresión de todas las velocidades calculadas
    for v_idx = 1:length(vel_vector)
        fprintf('Velocidad en el frame %.0f: %.2f km/h\n', v_idx, vel_vector(v_idx));
    end

    % Impresión de la velocidad promedio del objeto
    fprintf('\nVelocidad promedio del objeto: %.2f km/h\n', mean(vel_vector));
end