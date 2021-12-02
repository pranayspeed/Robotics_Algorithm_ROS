
function cornerness_img = harris(image, corners_count)

k = 0.04;

%x derivative
sobelx = 1/16 * [1 4 6 4 1]' * [-1 0 1];

%y derivative
sobely = sobelx';


%To get second derivative differentiate twice.
Ixx = convolute2d(convolute2d(image, sobelx),sobelx);
%Iyy  
Iyy = convolute2d(convolute2d(image, sobely),sobely);
%Ixy Image 
Ixy = convolute2d(convolute2d(image, sobelx),sobely);


%Get Determinant and trace 
det = Ixx.*Iyy - Ixy.*Ixy;
trace = Ixx + Iyy;

%Harris
R_val = det - k * trace;

sorted = sort(R_val(:));
nth = sorted(end-corners_count);

R_ind  = R_val>nth;

cornerness_img = R_ind;
end



function conv_image = convolute2d(image, kernel)

[irows, icols] = size(image);
[krows, kcols] = size(kernel);
conv_image = zeros(irows,icols);
krhalf = int32(krows/2);
kchalf = int32(kcols/2);
for y = krhalf+1 : irows-krhalf
    for x = kchalf+1 : icols-kchalf
        for jkern = 1:krows
            for ikern = 1:kcols
                kern_val = kernel(jkern, ikern);
                img_y = y+jkern-krhalf-1;
                img_x =  x+ikern-kchalf-1;
                conv_image(y, x) = conv_image(y, x) +image(img_y,img_x)*kern_val;
            end
        end
    end
end

end

