clc;clear

folderPath = '../build/controller_plots/';

csvFiles = dir(fullfile(folderPath, '*.csv'));

figure;
hold on;
for i = 1:length(csvFiles)
    filePath = fullfile(folderPath, csvFiles(i).name);
    data = readmatrix(filePath);
    x = data(:, 1);
    y = data(:, 2);
    
    plot(x, y, 'DisplayName', csvFiles(i).name);
end

xlabel('Czas');
ylabel('Wartość regulowana');
title('Test regulatorów');
legend('Location', 'Best');

hold off;
