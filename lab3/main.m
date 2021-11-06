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

    subplot(1,4,1);
    imshow(rgb_img);  
    title("image"+i+"-color");

    %%
    % Part 2 ================================================
    % Sum of Squared Differences

     window_size =30;
     pad_size=30;
     ssd_image = im_align1(r, g, b, window_size,pad_size);
     figure(i);

     subplot(1,4,2);
     imshow(ssd_image);
     title("image"+i+"-ssd");

    outfile_name = "image"+i+"-ssd.jpg";
    imwrite(ssd_image, outfile_name);

     %%
    % Part 3 ================================================
    % Normalized cross-correlation


     pad_size=20;
     ncc_image = im_align2(r, g, b, pad_size);
     figure(i);

     subplot(1,4,3);
     imshow(ncc_image);
     title("image"+i+"-ncc");

    outfile_name = "image"+i+"-ncc.jpg";
    imwrite(ncc_image, outfile_name);
 
    
    
     %%
    % Part 4 ================================================
    % Corner detection and alignment

     pad_size =60;
     cross_image = im_align3(r, g, b, pad_size);

     figure(i);

     subplot(1,4,4);
     imshow(cross_image);
     title("image"+i+"-corner");

    outfile_name = "image"+i+"-corner.jpg";
    imwrite(cross_image, outfile_name);


end




function [blueChannel,greenChannel,redChannel]= get_separate_BGR(img)

[height, ~] = size(img);

% Extract the individual blue, green, and red color channels.
blueChannel = img(1:height/3, :);
greenChannel = img(height/3+1:(2*height)/3, :);
redChannel = img((2*height/3)+1:height, :);

end







