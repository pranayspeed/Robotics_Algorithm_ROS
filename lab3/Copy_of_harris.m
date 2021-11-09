clc; clear all; close all
%===============================


%Image filter order is BGR
image_count = 1;

pad_size = 80;
for i=1:image_count

    img = imread("image"+i+".jpg");
    [b,g,r] = get_separate_BGR(img);
    
    pad_b = pad_image(b,pad_size);
    b_c = harris1(pad_b,200);
    [x,y] = find(b_c>0);

    corners_a = [x y]
    
    imshow(b_c);

end

function cornerness_img = harris1(image, corners_count)

im_dim = size(image);

%get gradient

k = 0.04;

%x derivative
%sobelx = [-1 0 1;-2 0 2;-1 0 1];
sobelx = 1/16 * [1 4 6 4 1]' * [-1 0 1];

%y derivative
sobely = sobelx';

%Get Ixx 
%To get second derivative differentiate twice.
Ixx = conv2(conv2(image, sobelx, "same"),sobelx,"same");     
%Iyy  
Iyy = conv2(conv2(image, sobely, "same"),sobely,"same");
%Ixy Image 
Ixy = conv2(conv2(image, sobelx, "same"),sobely,"same");


%Get Determinant and trace 
det = Ixx.*Iyy - Ixy.*Ixy;
trace = Ixx + Iyy;

%Harris
R_val = det - k * trace.*trace;

sorted = sort(R_val(:));
nth = sorted(end-corners_count);

R_ind  = R_val>nth;

%c_img  = R_val .* R_ind;
%cornerness_img = c_img>0;

cornerness_img = R_ind;


% cornerness_img = zeros(im_dim(1),im_dim(2));
% for i = window_size+1:im_dim(1) - window_size
%     for j = window_size+1:im_dim(2) -  window_size
%       curr_pix = image(i-window_size:i+window_size,j-window_size:j+window_size);
%       [gradx , grady] = gradient(curr_pix);
%       Hxy = [gradx*gradx gradx*grady;gradx*grady grady*grady];
%       
%       trace_H = gradx+grady;
%       R = det(Hxy) - k* (trace_H*trace_H);
%       R
%       if R > 0
%         R
%         cornerness_img(i,j) = R;
%       end
% %     bs=circshift(image,[i,j]);
% %     cur_error = SSD(a,bs);
% %     if cur_error < min_error
% %         min_error = cur_error;
% %         y_shift = i;
% %         x_shift = j;
% %     end
%     end
% end

end

function [x,y] = gradient(img)
img_d = im2double(img);
lambda_s = eigs(img_d,2);
x = lambda_s(1);
y = lambda_s(2);

end



function [blueChannel,greenChannel,redChannel]= get_separate_BGR(img)

[height, ~] = size(img);

h3 = floor(height/3);

% Extract the individual blue, green, and red color channels.
blueChannel = img(1:h3, :);
greenChannel = img(h3+1:2*h3, :);
redChannel = img(2*h3+1:3*h3, :);
end




function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));

end