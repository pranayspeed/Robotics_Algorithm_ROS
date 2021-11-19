function [feature_image, rgb_shift] = im_align3(r,g,b)
pad_size =50;
window_size = 40;

pad_r = pad_image(r,pad_size);
pad_g = pad_image(g,pad_size);
pad_b = pad_image(b,pad_size);


rgb_shift = zeros(3,2);

[x,y] = current_align(pad_r,pad_b, window_size);

if max(abs([x,y]))> window_size
    [gx,gy] = current_align(pad_g,pad_b, window_size);
    sg =my_circshift(g,[gy,gx]);

    rgb_shift(2,:) = [gy,gx];


    [rx,ry] = current_align(pad_r,pad_g, window_size);
    sr =my_circshift(r,[ry+gy,rx+gx]);

    rgb_shift(1,:) = [ry+gy,rx+gx];

else
    sr =my_circshift(r,[y,x]);

    rgb_shift(1,:) = [y,x];

    [x,y] = current_align(pad_g,pad_b, window_size);
    sg =my_circshift(g,[y,x]);

    rgb_shift(2,:) = [y,x];    
end


feature_image = cat(3, sr, sg, b);

rgb_shift(3,:) = [0 0];

end


function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));

end


function [x, y] =  current_align(a,b, window_size)
thresh_inliners =0.05;

[x,y] = ransac_align(a, b, window_size, thresh_inliners);

end

function [ x, y] = ransac_align(a, b, window_size, thresh_inliners)
x=0;
y=0;

num_features = 200;
%%==================================
% cornerness extraction

ca = harris(a,num_features);
[row,col] = find(ca>0);
corners_a = [row col];

cb = harris(b,num_features);
[row,col] = find(cb>0);
corners_b = [row col];

%%==================================

itercount = 0.2 * num_features;
thDist = 1; % inliner pixel window

sampleNum = 1;
min_inliners = round(thresh_inliners*num_features);


max_inliner = 0;

p=1;


while p <= itercount
	% fit using 2 random points
	sampleIdx_a = randsample(window_size,sampleNum);
	ptSample_a = corners_a(sampleIdx_a,:);
	sampleIdx_b = randsample(window_size,sampleNum);
	ptSample_b = corners_b(sampleIdx_b,:);
    
    % pixel shift
    d = ptSample_b-ptSample_a;
    
	% apply shift to all samples in b    
    s_corners_b = corners_b - d;
	
    % check shifted b with a for inliners
    [~, dist] = compute_closest_dists(corners_a, s_corners_b);
    inliers = find(abs(dist) < thDist);
    
    curr_inliners = length(inliers);
    if curr_inliners < min_inliners
        continue; 
    end
    
    if max_inliner < curr_inliners
        max_inliner = curr_inliners;
        y = int32(d(1));
        x = int32(d(2));
    end
    
    p=p+1;

end


end


function [indx, dist] = compute_closest_dists(a , b)

dist = zeros(size(b,1),1);
indx = zeros(size(b,1),1);
for i = 1:size(b,1) 
    yi = repmat(b(i,:),size(a,1),1);
    [dist(i),indx(i)] = min(sum((a-yi).^2,2));
    dist(i) = sqrt(dist(i));
end

end



function res = my_circshift(a,shifts)
    y = shifts(1);
    x = shifts(2);
    [ys, xs] = size(a);
    
    yarr = [ys - y+1:ys 1:ys-y];
    xarr = [xs - x+1:xs 1:xs-x];
    
    if y < 0
       yarr = [-y+1:ys  1:-y ]; 
    end
    if x < 0
        xarr = [-x+1:xs  1:-x ];
    end
    res = a(yarr, xarr);
    
end

