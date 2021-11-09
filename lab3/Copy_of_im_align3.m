function [feature_image, rgb_shift] = im_align3(r,g,b, pad_size)

pad_r = pad_image(r,pad_size);
pad_g = pad_image(g,pad_size);
pad_b = pad_image(b,pad_size);

rgb_shift = zeros(3,2);

[x,y] = ransac_align(pad_r,pad_b);
sr =circshift(r,[y,x]);

rgb_shift(1,:) = [y,x];

[x,y] = ransac_align(pad_g,pad_b);
sg =circshift(g,[y,x]);

rgb_shift(2,:) = [y,x];

feature_image = cat(3, sr, sg, b);

rgb_shift(3,:) = [0 0];

end


function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));

end



function [ x, y] = ransac_align(a, b)
x=0;
y=0;

num_features = 200;
%%==================================
% cornerness extraction
[row,col] = find(b_c>0)
ca = detectHarrisFeatures(a);
corners_a = ca.selectStrongest(num_features).Location;

cb = detectHarrisFeatures(b);
corners_b = cb.selectStrongest(num_features).Location;

num_features = min(length(corners_a), length(corners_b));
%%==================================

itercount = 7;
thDist = 2; % inliner pixel window
thInlrRatio = 0.20; %95% inliners
sampleNum = 1;
thInlr = round(thInlrRatio*num_features);
inlrNum = zeros(1,itercount);
x1 = zeros(1,itercount);
y1 = zeros(1,itercount);

p=1;

while p <= itercount
	% 1. fit using 2 random points
	sampleIdx_a = randsample(num_features,sampleNum);
	ptSample_a = corners_a(sampleIdx_a,:);
	sampleIdx_b = randsample(num_features,sampleNum);
	ptSample_b = corners_b(sampleIdx_b,:);
    
    % pixel shift
    d = ptSample_a-ptSample_b;
    
	% apply shift to all samples in b
    
    s_corners_b = corners_b + d;
	
    % check shifted b with a for inliners
    
    [k, dist] = dsearchn(corners_a, s_corners_b);
    
    inlier1 = find(abs(dist) < thDist);
    if length(inlier1) < thInlr
        continue; 
    end
    inlrNum(p) = length(inlier1);
    x1(p) = d(1);
    y1(p) = d(2);
    p=p+1;

end
% % 3. choose the coef with the most inliers
[max_val,idx] = max(inlrNum);
x = -int32(x1(idx));
y = -int32(y1(idx));


end

