% Converter.m
% Bohan Zhang, 2022
clear;
clc;

% Read png files from folder
imgFiles = dir('*.PNG');   

% Get len of file list
numFIle = length(imgFiles);

% Go through files
for idx = 1:numFIle
   % Get full name
   name = imgFiles(idx).name;
   
   % Read file
   currentImg = imread(name);
   
   % Separate key name
   nameContent = strsplit(name,".");
   trueName = nameContent(1); % get true name of file
   
   % Gaussin Noise
   imgGauss = imnoise(currentImg,'gaussian'); % content
   nameGauss = trueName + "Gauss"+".png"; % name
   imwrite(imgGauss,nameGauss); % write img

   % salt & pepper
   imgPeP = imnoise(currentImg,'salt & pepper'); % content
   namePeP = trueName + "PeP"+".png"; % name
   imwrite(imgPeP,namePeP); % write img

   % poisson
   imgPoi = imnoise(currentImg,'poisson'); % content
   namePoi = trueName + "Poi"+".png"; % name
   imwrite(imgPoi,namePoi); % write img

   % speckle
   imgSpk = imnoise(currentImg,'speckle'); % content
   nameSpk = trueName + "Spk"+".png"; % name
   imwrite(imgSpk,nameSpk); % write img

   % Contrast 
   imgCon = imadjust(currentImg,stretchlim(currentImg),[]); % content
   nameCon = trueName + "Con"+".png"; % name
   imwrite(imgCon,nameCon); % write img

   % Saturation
   imgSat = imtweak(currentImg,'hsv',[0 1.5 1]); % content
   nameSat = trueName + "Sat"+".png"; % name
   imwrite(imgSat,nameSat); % write img

   % HSL
   imgHSL = imtweak(currentImg,'hsl',[0 1.5 1]); % content
   nameHSL = trueName + "HSL"+".png"; % name
   imwrite(imgHSL,nameHSL); % write img
   
end

%% Read txt files from folder
txtfiles = dir('*.txt'); 

% Get len of file list
numFIle = length(txtfiles);

% Go through files
for idx = 1:numFIle
   % Get full name
   name = txtfiles(idx).name;

   % Separate key name
   nameContent = strsplit(name,".");
   trueName = nameContent(1); % get true name of file

   % Gaussin Noise
   nameGauss = trueName + "Gauss"+".txt"; % name
   copyfile(name, nameGauss);

   % salt & pepper
   namePeP = trueName + "PeP"+".txt"; % name
   copyfile(name, namePeP);

   % poisson
   namePoi = trueName + "Poi"+".txt"; % name
   copyfile(name, namePoi);

   % speckle
   nameSpk = trueName + "Spk"+".txt"; % name
   copyfile(name, nameSpk);

   % Contrast
   nameCon = trueName + "Con"+".txt"; % name
   copyfile(name, nameCon);

   % Saturation
   nameSat = trueName + "Sat"+".txt"; % name
   copyfile(name, nameSat);

   % HSL
   nameHSL = trueName + "HSL"+".txt"; % name
   copyfile(name, nameHSL);
end
