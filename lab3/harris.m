
function cornerness_img = harris(image, corners_count)

%get gradient
%I = imgaussfilt(image,2);
gh = fspecial('gaussian',3,2);
k = 0.04;

%x derivative
%sobelx = [-1 0 1;-2 0 2;-1 0 1];
sobelx = 1/16 * [1 4 6 4 1]' * [-1 0 1];

%y derivative
sobely = sobelx';

I = conv2(image,gh,"same");
%Get Ixx 
%To get second derivative differentiate twice.
Ixx = conv2(conv2(I, sobelx, "same"),sobelx,"same");
%Iyy  
Iyy = conv2(conv2(I, sobely, "same"),sobely,"same");
%Ixy Image 
Ixy = conv2(conv2(I, sobelx, "same"),sobely,"same");


%Get Determinant and trace 
det = Ixx.*Iyy - Ixy.*Ixy;
trace = Ixx + Iyy;

%Harris
R_val = det - k * trace.*trace;

sorted = sort(R_val(:));
nth = sorted(end-corners_count);

R_ind  = R_val>nth;

cornerness_img = R_ind;

end
