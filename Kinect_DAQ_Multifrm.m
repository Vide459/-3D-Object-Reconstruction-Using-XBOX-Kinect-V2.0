function [ColorIMG, DepthIMG] = Kinect_DAQ_Multifrm(Vid_color, Vid_depth, nFrames, nPoses)

%This function is used to acquire multiple frames in different positions
%simultaneously. It takes into account the no. of. positions and the total
%number of frames per position and returns a total set of Depth Maps and
%Color Images for different Poses of the Object Of Interest.

Vid_color.FramesPerTrigger = 1;
Vid_depth.FramesPerTrigger = 1;

Vid_color.TriggerRepeat = nFrames;
Vid_depth.TriggerRepeat = nFrames;

triggerconfig([Vid_color Vid_depth],'manual');

for Pose_Count = 1 : nPoses

    start([Vid_color Vid_depth]);

    for i = 1 : nFrames

        trigger([Vid_color Vid_depth]);

        [ColorIMG{Pose_Count,i}, ~, ~] = getdata(Vid_color);
        [DepthIMG{Pose_Count,i}, ~, ~] = getdata(Vid_depth);

        figure;
        imshow(ColorIMG{Pose_Count,i});

        figure;
        imshow(DepthIMG{Pose_Count,i},[]);

        filename = datestr(now,'dd-mm-yy_HHMMSS');

        imwrite(ColorIMG{Pose_Count,i},strcat(filename,'_',num2str(Pose_Count),'_c.tif'));
        imwrite(DepthIMG{Pose_Count,i},strcat(filename,'_',num2str(Pose_Count),'_d.tif'));

    end

    stop([Vid_color Vid_depth]);

    if Pose_Count < nPoses

        pause;

    end

end

end