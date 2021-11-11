function [feature_image, rgb_shift] = im_align2(r,g,b)


pad_size =30;
window_size = 30;

pad_r = pad_image(r,pad_size);
pad_g = pad_image(g,pad_size);
pad_b = pad_image(b,pad_size);


rgb_shift = zeros(3,2);


[x,y] = current_align(pad_r,pad_g, window_size);
sr =circshift(r,[y,x]);

rgb_shift(1,:) = [y,x];

[x,y] = current_align(pad_b,pad_g, window_size);
sb =circshift(b,[y,x]);

rgb_shift(3,:) = [y,x];

feature_image = cat(3, sr, g, sb);

rgb_shift(2,:) = [0 0];

end


function [x, y] =  current_align(a,b, window_size)

ysize = size(a,1);
xsize = size(a,2);
c = normxcorr2(a, b);

[ypeak,xpeak] = find(c==max(c(:)));

y = ypeak-ysize;
x = xpeak-xsize;

end


function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
     
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));
   %img = padarray(image,[3 3],255,'both');
end

