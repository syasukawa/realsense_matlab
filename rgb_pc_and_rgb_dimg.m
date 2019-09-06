% Make Pipeline object to manage streaming
pipe = realsense.pipeline();

% Make Colorizer object to prettify depth output 
colorizer = realsense.colorizer();

% define point cloud object
pcl_obj = realsense.pointcloud();

% Start streaming with default settings
profile = pipe.start();

align_to = realsense.stream.color;
alignedFs = realsense.align(align_to);

% Get streaming device's name
dev = profile.get_device();
name = dev.get_info(realsense.camera_info.name);

% Get frames. We discard the first couple to allow
% the camera time to settle

for i = 1:5
fs = pipe.wait_for_frames();
end

%create a point cloud player
% player = pcplayer(xlimits,ylimits,zlimits)
player1 = pcplayer([-1 1],[-1 1],[-1 1]);
frameCount =0

while isOpen(player1) && frameCount < 2000
  frameCount = frameCount+1;
  fs = pipe.wait_for_frames();

  %align the depth frames to the color stream
  aligned_frames = alignedFs.process(fs);
  depth = aligned_frames.get_depth_frame();   
  color = fs.get_color_frame();
  c_depth = colorizer.colorize(depth);
  c_depthdata = c_depth.get_data();
  
  %get the points cloud based on the aligned depth stream
  pnts = pcl_obj.calculate(depth);
  pcl_obj.map_to(color);
  colordata = color.get_data();
  
  colordatavector = [colordata(1:3:end)',colordata(2:3:end)',colordata(3:3:end)'];
  vertices = pnts.get_vertices(); 
  view(player1,vertices,colordatavector) 

  %% image
  depth_img = permute(reshape(c_depthdata,[3,c_depth.get_width(),c_depth.get_height()]),[3 2 1]);
  depth_img_quarter = imresize(depth_img,0.25);
  
  depth_gimg = permute(reshape(depth.get_data(),[1, c_depth.get_width(),c_depth.get_height()]),[3 2 1]);
  depth_gimg_quarter = imresize(depth_gimg,0.25);
  
  c_img = permute(reshape(colordata,[3,color.get_width(),color.get_height()]),[3 2 1]);
  c_img_quarter = imresize(c_img,0.25);

  figure(2); 
  image(depth_img_quarter); 
  figure(3); 
  g_depthdata=uint8((double(depth_gimg_quarter)/10));
  imshow(g_depthdata); 
  figure(4);
  imshow(c_img_quarter); 
  
end

% Stop streaming
pipe.stop();