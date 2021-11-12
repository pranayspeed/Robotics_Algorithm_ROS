clc; clear all; close all
%===============================


%Image filter order is BGR
image_count = 6;

%% Images with the pixel shift as a title
% For each image the title of each sub image constains the alignment method
% along with the corrosponding RGB shifts
%% Note: Shifted R and B channels with G channel as fixed.

for i=1:image_count
%%    
    img = imread("image"+i+".jpg");
    [b,g,r] = get_separate_BGR(img);
    
    figure(i);

    % Part 1 ================================================
    % Recombine separate color channels into an BGR image.
    % 

    rgb_image = cat(3, r, g, b);

    outfile_name = "image"+i+"-color.jpg";
    imwrite(rgb_image, outfile_name);

    rgb_img = imread(outfile_name);
    
    subplot(2,2,1);
    imshow(rgb_img);  
    title("image"+i+"-color");

   
    % Part 2 ================================================
    % Sum of Squared Differences

     [ssd_image, rgbshift_ssd] = im_align1(r, g, b);

     subplot(2,2,2);
     imshow(ssd_image);
          
     img_title_ssd = rgb_shift_print("SSD", rgbshift_ssd);

     title(img_title_ssd);
    outfile_name = "image"+i+"-ssd.jpg";
    imwrite(ssd_image, outfile_name);

    
    % Part 3 ================================================
    % Normalized cross-correlation

     [ncc_image, rgbshift_ccn ] = im_align2(r, g, b);

     subplot(2,2,3);
     imshow(ncc_image);
     
    img_title_ncc = rgb_shift_print("NCC", rgbshift_ccn);

    title(img_title_ncc);
    outfile_name = "image"+i+"-ncc.jpg";
    imwrite(ncc_image, outfile_name);
    
    
    % Part 4 ================================================
    % Corner detection and alignment

     pad_size =40;
     [cross_image, rgbshift_corner] = im_align3(r, g, b);

     subplot(2,2,4);
     
     imshow(cross_image);
     
    img_title_corner = rgb_shift_print("Corner", rgbshift_corner);

    title(img_title_corner);
    outfile_name = "image"+i+"-corner.jpg";
    imwrite(cross_image, outfile_name);
    
    drawnow
    %% RGB Shifts
    fprintf("image"+i+" "+ img_title_ssd);
    fprintf("image"+i+" "+ img_title_ncc);
    fprintf("image"+i+" "+ img_title_corner);
    %%

end

%%


function [blueChannel,greenChannel,redChannel]= get_separate_BGR(img)

[height, ~] = size(img);

h3 = floor(height/3);

% Extract the individual blue, green, and red color channels.
blueChannel = img(1:h3, :);
greenChannel = img(h3+1:2*h3, :);
redChannel = img(2*h3+1:3*h3, :);
end

function output = rgb_shift_print(method, rgbshift)

output = sprintf('%8s RGB Shift  R(%3d,%3d),G(%3d,%3d),B(%3d,%3d)\n',method, rgbshift(1,:),rgbshift(2,:),rgbshift(3,:));

end






