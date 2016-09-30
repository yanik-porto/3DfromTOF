function depth_img = depth_map_from_cloud(cloud, sz)

for i = 1:sz(1)
    for j = 1:sz(2)
        depth_img(i,j) = cloud(3,(i-1)*320+j);
    end
end

depth_img(isnan(depth_img)) = 0;




