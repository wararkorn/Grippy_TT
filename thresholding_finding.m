chic = imread('.\pic\dum.png');
hsv = rgb2hsv(chic);

subplot(1,3,1)
imhist(hsv(:,:,1))

subplot(1,3,2)
imhist(hsv(:,:,2))

subplot(1,3,3)
imhist(hsv(:,:,3))

bi = (hsv(:,:,1) <= 0.25) & (hsv(:,:,2))  & (hsv(:,:,3) <= 0.8);

