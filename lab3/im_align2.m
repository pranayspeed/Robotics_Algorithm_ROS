function ncc_image = im_align2(r,g,b, pad_size)

pad_r = pad_image(r,pad_size);
pad_g = pad_image(g,pad_size);
pad_b = pad_image(b,pad_size);
ysize = size(pad_r,1)
xsize = size(pad_r,2)
c = normxcorr2(pad_r, pad_g);

[ypeak,xpeak] = find(c==max(c(:)));
sr =circshift(r,[ypeak-ysize,xpeak-xsize]);

c = normxcorr2(pad_b, pad_g);

[ypeak,xpeak] = find(c==max(c(:)));

sb =circshift(b,[ypeak-ysize,xpeak-xsize]);

ncc_image = cat(3, sr, g, sb);

end


function img = pad_image(image, pad_size)
   [row,col] = size(image);
   corp_size=pad_size;
   img = image(corp_size/2 : row - (corp_size/2) , corp_size/2 : col - (corp_size/2));
   size(img)
end

