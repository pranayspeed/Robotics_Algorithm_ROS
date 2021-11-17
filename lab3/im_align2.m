function [feature_image, rgb_shift] = im_align2(r,g,b)

pad_size =40;
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

[x,y] = align_2_image_ncc(a, b, window_size);

end

function [x_shift, y_shift] = align_2_image_ncc(a,b, window_size)

adb = im2double(a);
bdb = im2double(b);

% Normalized image pixels
an = adb./norm(adb);
bn = bdb./norm(bdb);

x_shift = 0;
y_shift = 0;
max_error = 0;
for i = -window_size:window_size
    for j = -window_size:window_size
    bs=circshift(bn,[i,j]);
    cur_error = sum(dot(an,bs));% Normalized cross-correlation
    if cur_error > max_error
        max_error = cur_error;
        y_shift = -i;
        x_shift = -j;
    end
    end
end

end

function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
     
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));
end





