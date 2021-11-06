function [ssd_image, rgb_shift] = im_align1(r,g,b, window_size, pad_size)

pad_r = pad_image(r,pad_size);
pad_g = pad_image(g,pad_size);
pad_b = pad_image(b,pad_size);


rgb_shift = zeros(3,2);

[sx, sy] = align_2_image_ssd(pad_g,pad_r,window_size);

sr =circshift(r,[sy,sx]);

rgb_shift(1,:) = [sy,sx];

[sx, sy] = align_2_image_ssd(pad_g,pad_b,window_size);

sb =circshift(b,[sy,sx]);

rgb_shift(3,:) = [sy,sx];

ssd_image = cat(3, sr, g, sb);



rgb_shift(2,:) = [0 0];

end


function [x_shift, y_shift] = align_2_image_ssd(a,b, window_size)

x_shift = 0;
y_shift = 0;
min_error = inf;
for i = -window_size:window_size
    for j = -window_size:window_size
    bs=circshift(b,[i,j]);
    cur_error = SSD(a,bs);
    if cur_error < min_error
        min_error = cur_error;
        y_shift = i;
        x_shift = j;
    end
    end
end

end


function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));

end

function ssd = SSD(a,b)
    ssd = sum(sum((double(a) - double(b)) .^ 2));
end 
