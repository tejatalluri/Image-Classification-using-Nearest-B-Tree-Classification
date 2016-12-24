function colorMoments = colorMoments(image)


%% extract color channels
R = double(image(:, :, 1));
G = double(image(:, :, 2));
B = double(image(:, :, 3));

%% compute 2 first color moments from each channel
meanR = mean( R(:) );
stdR  = std( R(:) );
meanG = mean( G(:) );
stdG  = std( G(:) );
meanB = mean( B(:) );
stdB  = std( B(:) );

%% construct output vector
colorMoments = zeros(1, 6);
colorMoments(1, :) = [meanR stdR meanG stdG meanB stdB];



end