warning off;
close all;
clear all;
clc;

data


%% To select Image


[filename, pathname] = uigetfile('*.jpg', 'Pick an Image');
if isequal(filename,0) | isequal(pathname,0)
      warndlg('Image is not selected');
else
    a=imread(filename);
    figure(1),imshow(a);
    title('Input Image');
end

rng('default');
%% rand pont generation
P = rand(2, 50);

%% Set point

X = rand(2, 50);

%% nearest neighbour

I = nearestneighbour(P, X);


%% Plot the points

figure(2)
plot(X(1,:), X(2,:), 'b.', P(1,:), P(2,:), 'r.', 'MarkerSize', 15);
hold on
quiver(P(1, :), P(2, :), X(1,I) - P(1, :), X(2, I) - P(2, :), 0, 'k');
hold off

%% Nearest neighbour to subset of a point set

X = rand(4, 20);

%% Find nearest neighbour  features

I = nearestneighbour([2 4 10], X);

%%  Find nearest neighbours to all columns 

I = nearestneighbour(X);

%% Example 3: Finding nearest n neighbours

P = rand(2, 3);

%% Candidate feature set

X = rand(2, 50);

%% Calculate the 4 nearest neighbours to each point: I(:, 1) will be the 4


I = nearestneighbour(P, X, 'NumberOfNeighbours', 4);

%%  Plot the points - show the neighbours for one of them

figure(3)
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on
p1 = repmat(P(1,1), 1, 4); p2 = repmat(P(2,1), 1, 4);
quiver(p1, p2, X(1, I(:, 1)) - p1, X(2, I(:, 1)) - p2, 0, 'k')
hold off

%% Example 4: Finding neighbours within a specified radius

%% three points of interest

P = rand(2, 3);

%%  Candidate feature set

X = rand(2, 50);

I = nearestneighbour(P, X, 'Radius', 0.2);

%% Plot the points - show the neighbours for one of them

idx = I(I(:, 1) ~= 0, 1);
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on
p1 = repmat(P(1,1), 1, length(idx)); p2 = repmat(P(2,1), 1, length(idx));
quiver(p1, p2, X(1, idx) - p1, X(2, idx) - p2, 0, 'k')
hold off

%% Combining NumberOfNeighbours and Radius

%% Point of interest
P = rand(2, 1);

% Candidate feature set
X = rand(2, 30);

%% Calculate the  nearest neighbours Euclidean distance

I = nearestneighbour(P, X, 'n', 5, 'r', 0.2);
n = length(I);

%% Plot the points - show the neighbours for one of them
figure(4)
plot(P(1,:), P(2, :), 'r.', X(1,:), X(2,:), 'b.', 'MarkerSize', 15)
hold on

quiver(repmat(P(1), 1, n), repmat(P(2), 1, n), X(1, I) - P(1), X(2, I) - P(2), 0, 'k')
hold off


%% Large Data set, few points to look up
P = rand(3, 10);
X = rand(3, 5000);
%% To load database
%% Image Search

[X1] = imread(filename);
HSVmap1 = rgb2ycbcr(X1);

%%  Open database txt file... for reading...

fid = fopen('database.txt');

%% Results matrix...

resultValues = [];      
resultNames = {};

%%  Indices...

i = 1;                  
j = 1;

while 1
    imagename = fgetl(fid);
    if ~ischar(imagename), break, end       
     [X] = imread(imagename);
     HSVmap = rgb2ycbcr(X);
    [D1,D2,D3] = quadratic1(X1,  HSVmap1, X, HSVmap);
    resultValues1(i) = D1;
    resultValues2(i) = D2;
    resultValues3(i) = D3;
    resultNames(j) = {imagename};
    i = i + 1;
    j = j + 1;
end
fclose(fid);


%% Sorting colour results...

[sortedValues1, index1] = sort(resultValues1);     
[sortedValues2, index2] = sort(resultValues2); 
[sortedValues3, index3] = sort(resultValues3); 

%% color Features

[rows columns numberOfColorBands] = size(a);
hsvHist = hsvHistogram(a);
color_moments = colorMoments(a);

%% local features 

a1=rgb2gray(a);
corners = detectFASTFeatures(a1,'MinContrast',0.1);
J = insertMarker(a1,corners,'circle');
figure(5),imshow(J);
title('Local Features');


%% local Image descriptors


points = detectSURFFeatures(a1);
[features, valid_points] = extractFeatures(a1, points);
figure(6); imshow(a1); hold on;
plot(valid_points.selectStrongest(20),'showOrientation',true);
title('local Image descriptors');

%% Entropy

E = entropy(a1);

%% correlation

J = stdfilt(a1);
R = corr2(a1,J);

%% Region Based Features

s  = regionprops(a1, 'centroid');
centroids = cat(1, s.Centroid);
figure(7),imshow(a1)
hold on
plot(centroids(:,1), centroids(:,2), 'g*')
title('Region Based Features');
hold off

%% SIFT Algorithm

%% get the Key Points

Options.upright=true;
Options.tresh=0.0001;
Ipts1=OpenSurf(a,Options);
Ipts2=OpenSurf(a,Options);

%% Put the landmark descriptors in a matrix

 D1 = reshape([Ipts1.descriptor],64,[]); 
 D2 = reshape([Ipts2.descriptor],64,[]); 
 
 
 %% Find the best matches
 
 err=zeros(1,length(Ipts1));
 cor1=1:length(Ipts1); 
 cor2=zeros(1,length(Ipts1));
  for i=1:length(Ipts1),
      distance=sum((D2-repmat(D1(:,i),[1 length(Ipts2)])).^2,1);
      [err(i),cor2(i)]=min(distance);
  end

%% Sort matches on vector distance

[err, ind]=sort(err); 
cor1=cor1(ind); 
cor2=cor2(ind);

%% Show both images

  I = zeros([size(a,1) size(a,2)*2 size(a,3)]);
  I(:,1:size(a,2),:)=a; I(:,size(a,2)+1:size(a,2)+size(a,2),:)=a;
  figure(8), imshow(I/255); hold on;
  title('SIFT Images');
%% Show the best matches
  for i=1:30,
      c=rand(1,3);
      plot([Ipts1(cor1(i)).x Ipts2(cor2(i)).x+size(a,2)],[Ipts1(cor1(i)).y Ipts2(cor2(i)).y],'-','Color',c)
      plot([Ipts1(cor1(i)).x Ipts2(cor2(i)).x+size(a,2)],[Ipts1(cor1(i)).y Ipts2(cor2(i)).y],'o','Color',c)
  end
 
%% Search Images

fid = fopen('colourResults_R_C.txt', 'w+');         % Create a file, over-write old ones.

for i = 1:20                                        % Store top 10 matches...

    tempstr = char(resultNames(index1(i)));
    fprintf(fid, '%s\r', tempstr);
    
    disp(resultNames(index1(i)));
    disp(sortedValues1(i));
    disp('  ');
end

fclose(fid);

%% Retrieval Images

filename='colourResults_R_C.txt';
fid = fopen(filename);
i=1;
while 1
imagename = fgetl(fid);
if ~ischar(imagename), break, end       
[x, map] = imread(imagename);
figure(7),imshow(x);
title('Retrieval Images');
subplot(4,5,i);
i=i+1;
figure(9),imshow(x);
title('Retrieval Images');
end

fclose(fid);


