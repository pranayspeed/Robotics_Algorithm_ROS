clc; clear all; close all
%===============================


%% Image Channel Alignment using SSD
%%
% 
% # Image is splitted in 3 equal channels B,G and R. 
% # 20 boundary pixels are cropped before the Image processed for alignment.
% # Used a displacemet window of [-30, 30] for alignment search.
% # Using channel G as fixed, search for x,y shift with minimum SSD value for R
% channel looping within the displacement window.
% # Repeat the previous step for channel B, keeping G as fixed.
% # Concatenate the channels R,G,B applying the shifts calculated and save the image 
%


%% Image Channel Alignment using NCC
%%
% 
% # Image is splitted in 3 equal channels B,G and R. 
% # 15 boundary pixels are cropped before the Image processed for alignment.
% # Using channel G as fixed, search for x,y shift with maximum/peak value 
% for normalized cross-correlation between R and fixed channel G.
% # Repeat the previous step for channel B, keeping G as fixed.
% # Concatenate the channels R,G,B applying the shifts calculated and save the image 
%

%% Image Channel Alignment using Corner Feature detection
%%
% 
% # Image is splitted in 3 equal channels B,G and R. 
% # 20 boundary pixels are cropped before the Image processed for alignment.
% # Using channel G as fixed, search for x,y shift using top 200 corners 
% extracted using harris corner detection function defined in harris.m file
% and then using this corners with RANSAC algorithm to find the best alignment
% by selecting the maximum inliners alignment shift value.
% # Repeat the previous step for channel B, keeping G as fixed.
% # Concatenate the channels R,G,B applying the shifts calculated and save the image 
%


%% Image Channel Alignment Output for SSD, NCC and Corners
% * For each image the title of each sub image constains the alignment method
% along with the corrosponding RGB shifts
% *Note: Shifted R and B channels with G channel as fixed.*

%Image filter order is BGR
image_count = 6;

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






