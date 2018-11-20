function [ColorIMG, DepthIMG] = Kinect_DAQ_Multifrm_Env(Vid_color, Vid_depth, nframes)

%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

Vid_color.FramesPerTrigger = 1;
Vid_depth.FramesPerTrigger = 1;

Vid_color.TriggerRepeat = nframes;
Vid_depth.TriggerRepeat = nframes;

triggerconfig([Vid_color Vid_depth],'manual');

start([Vid_color Vid_depth]);

for i = 1 : nframes
    
    trigger([Vid_color Vid_depth]);

    [ColorIMG{i}, ~, ~] = getdata(Vid_color);
    [DepthIMG{i}, ~, ~] = getdata(Vid_depth);

    figure;
    imshow(ColorIMG{i});

    figure;
    imshow(DepthIMG{i},[]);

    filename = datestr(now,'dd-mm-yy_HHMMSS');
 
    imwrite(ColorIMG{i},strcat(filename,'_Env_c.tif'));
    imwrite(DepthIMG{i},strcat(filename,'_Env_d.tif'));
    
end

stop([Vid_color Vid_depth]);

end