clc; clear all; close all
%===============================


%Image filter order is BGR
image_count = 6;


for i=1:image_count

    img = imread("image"+i+".jpg");
    [b,g,r] = get_separate_BGR(img);

    %%
    % Part 1 ================================================
    % Recombine separate color channels into an BGR image.
    % 
    rgb_image = cat(3, r, g, b);

    outfile_name = "image"+i+"-color.jpg";
    imwrite(rgb_image, outfile_name);

    rgb_img = imread(outfile_name);
    % for debugging the output images
    figure(i)

    subplot(2,2,1);
    imshow(rgb_img);  
    title("image"+i+"-color");

    %%
    % Part 2 ================================================
    % Sum of Squared Differences

     window_size =30;
     pad_size=30;
     [ssd_image, rgbshift_ssd] = im_align1(r, g, b, window_size,pad_size);
     figure(i);

     subplot(2,2,2);
     imshow(ssd_image);
     
     
     out_shift = rgb_shift_print("SSD", rgbshift_ssd);
     title("image"+i+" "+ out_shift);
    outfile_name = "image"+i+"-ssd.jpg";
    imwrite(ssd_image, outfile_name);

     %%
    % Part 3 ================================================
    % Normalized cross-correlation


     pad_size=20;
     [ncc_image, rgbshift_ccn ] = im_align2(r, g, b, pad_size);
     figure(i);

     subplot(2,2,3);
     imshow(ncc_image);
     
    out_shift = rgb_shift_print("CCN", rgbshift_ccn);
    title("image"+i+" "+ out_shift);
    outfile_name = "image"+i+"-ncc.jpg";
    imwrite(ncc_image, outfile_name);
 
    
    
     %%
    % Part 4 ================================================
    % Corner detection and alignment

     pad_size =60;
     [cross_image, rgbshift_corner] = im_align3(r, g, b, pad_size);

     figure(i);

     subplot(2,2,4);
     imshow(cross_image);

    out_shift = rgb_shift_print("Corner", rgbshift_corner);
    title("image"+i+" "+ out_shift);
    outfile_name = "image"+i+"-corner.jpg";
    imwrite(cross_image, outfile_name);


end




function [blueChannel,greenChannel,redChannel]= get_separate_BGR(img)

[height, ~] = size(img);

h3 = floor(height/3);

% Extract the individual blue, green, and red color channels.
blueChannel = img(1:h3, :);
greenChannel = img(h3+1:2*h3, :);
redChannel = img(2*h3+1:3*h3, :);
end

function output = rgb_shift_print(method, rgbshift)

% Shifted  R G B
%disp(['RGB Shift  R(',rgbshift(1,:),'), G(',rgbshift(2,:),'), B(',rgbshift(3,:),').']);

output = sprintf('%s RGB Shift  R(%d,%d),G(%d,%d),B(%d,%d)\n',method, rgbshift(1,:),rgbshift(2,:),rgbshift(3,:));

fprintf(output);
end






