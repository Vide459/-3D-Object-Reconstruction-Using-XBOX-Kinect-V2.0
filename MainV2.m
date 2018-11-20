clc;
clear all;
close all;

%%

load Kinect_Depth_Intrinsics;
% load Kinect_Color_Intrinsics;

DAQ = input('Do you want to acquire data? If yes press "1" or else press "0" >>> ');

if (DAQ == 1)
        
    info = imaqhwinfo('kinect');

    disp(info);

    Color_Video = videoinput('kinect', 1);
    Depth_Video = videoinput('kinect', 2);
    
    nPoses = input('Enter the total number of poses( an "ODD" number ) >>> ');
    
    while isinteger(nPoses/2)
        
        nPoses = input('Enter the total number of poses( an "ODD" number ) >>> ');
        
    end
    
    disp('Capture the frames in such a way that the central frame in the series will be the final position of the required point cloud.')
    
    nFrames = input('Enter the number of frames acquired per pose >>> ');
    
    nFrames_env = input('Enter the number of frames acquired for empty environment >>> ');
    
    disp('Capture the empty environment intially.');
    
    pause;
    
    [ColorIMG_Env, DepthIMG_Env] = Kinect_DAQ_Multifrm_Env(Color_Video, Depth_Video, nFrames_env);
    
    disp('Place the Object of interest.');
    
    pause;
    
    disp('Please measure the distance between the object and the Kinect sensor.');
       
    Kinect2Obj_Distance = input('Enter the distance between the Kinect sensor and the Object>>> ');
    
    [ColorIMG, DepthIMG] = Kinect_DAQ_Multifrm(Color_Video, Depth_Video, nFrames, nPoses);
    
elseif (DAQ == 0)
    
    Kinect2Obj_Distance = input('Enter the distance between the Kinect sensor and the Object>>> ');
    
    nPoses = input('Enter the total number of poses >>> ');
    
end

%%

if (DAQ == 0)
    
    filename_end = strcat('*_Env_d.tif');
    Environment_images = dir(filename_end);

    EnvIMG_count = length(Environment_images);

    for i = 1 : EnvIMG_count
        filename = Environment_images(i).name;
        currentimage = imread(filename);

        DepthIMG_Env{i} = currentimage;

        figure;
        imshow(DepthIMG_Env{i},[]);
    end
end

[r_env,c_env,p_env] = size(DepthIMG_Env{1});
EnvIMG_count = length(DepthIMG_Env);

ENV_avg = uint16(zeros(r_env,c_env,p_env));

for i = 1 : r_env
    
    for j = 1 : c_env
        
        for k1 = 1 : EnvIMG_count
            
           X1(k1) = double(DepthIMG_Env{k1}(i,j));
           
        end
        
        Stndrd_dev1 = std(X1);
        X1_m = median(X1);
        
        a = 0;
        
        for k2 = 1 : EnvIMG_count
            if (abs(X1(k2) - X1_m) <= (2.5*Stndrd_dev1))
                
                a = a + 1;
                X_inlier1(a) = X1(k2);
                
            end
        end
        
        Inlier_count1 = length(X_inlier1);
        
        for k3 = 1 : Inlier_count1
            
            ENV_avg(i,j) = ENV_avg(i,j) + X_inlier1(k3);
            
        end
        
        if ENV_avg(i,j) ~= 0
            
            ENV_avg(i,j) = ENV_avg(i,j) ./ Inlier_count1;
            
        end
        
    end
    
end

figure;
imshow(ENV_avg,[]);

ENV_HF = imfill(ENV_avg,8);
    
figure;
imshow(ENV_HF,[]);

filename = datestr(now,'dd-mm-yy_HHMMSS');

imwrite(ENV_HF,strcat(filename,'_d_ENV_ENH_','median','.tif'));

%%

for Pose_Count = 1 : nPoses
    
    if (DAQ == 0)
        
        filename_end = strcat('*_',num2str(Pose_Count),'_d.tif');
        Depth_images = dir(filename_end);

        DepthIMG_count = length(Depth_images);

        for i = 1 : DepthIMG_count
            
            filename = Depth_images(i).name;
            currentimage = imread(filename);

            DepthIMG{Pose_Count,i} = currentimage;

            figure;
            imshow(DepthIMG{Pose_Count,i},[]);
            
        end
        
        [r,c,p] = size(DepthIMG{Pose_Count,1});
        [~,DepthIMG_count] = size(DepthIMG);
 
    else
        
        [r,c,p] = size(DepthIMG{Pose_Count,1});
        DepthIMG_count = nFrames;

    end
    
    Depth_Object_avg = uint16(zeros(r,c,p));
        
    OOI = uint16(zeros(r,c,p));

    for i = 1 : r

        for j = 1 : c

            for k = 1 : DepthIMG_count

                X(k) = double(DepthIMG{Pose_Count,k}(i,j));

            end

            Stndrd_dev = std(X);
            X_m = median(X);
            
            a = 0;

            for k = 1 : DepthIMG_count
                if (abs(X(k) - X_m) <= (2.5*Stndrd_dev))

                    a = a + 1;
                    X_inlier(a) = X(k);

                end
            end

            Inlier_count = length(X_inlier);

            for k = 1 : Inlier_count

                Depth_Object_avg(i,j) = Depth_Object_avg(i,j) + X_inlier(k);

            end

            if Depth_Object_avg(i,j) ~= 0

                Depth_Object_avg(i,j) = Depth_Object_avg(i,j) ./ Inlier_count;

            end

        end

    end

    figure;
    imshow(Depth_Object_avg,[]);
    
    Depth_Object_Avg{Pose_Count} = Depth_Object_avg;
    
    Depth_Object_Hf = imfill(Depth_Object_avg,8);
    
    Depth_Object_HF{Pose_Count} = Depth_Object_Hf;
    
    ooi = ENV_HF - Depth_Object_HF{Pose_Count};
        
    figure;
    imshow(Depth_Object_Hf,[]);
    
    figure;
    imshow(ooi,[]);
    
    for i = 1 : r
        for j = 1 : c
            if ((ooi(i,j) >= 100))
                OOI(i,j) = Depth_Object_Hf(i,j);
            end
        end
    end
    
    Depth_ooi{Pose_Count} = OOI;
    
    figure;
    imshow(OOI,[]);
    
    filename = datestr(now,'dd-mm-yy_HHMMSS');
    
    imwrite(OOI,strcat(filename,'_',num2str(Pose_Count),'_d_OOI_','median','.tif'));
    
    imwrite(Depth_Object_Hf,strcat(filename,'_',num2str(Pose_Count),'_d_ENH_','median','.tif'));
       
    PointCloud{Pose_Count} = Depth2PointCloud(Depth_Object_HF{Pose_Count},  Kinect2Obj_Distance, Kinect_Depth_Calib);
    
    PointCloud_OOI{Pose_Count} = Depth2PointCloud(Depth_ooi{Pose_Count}, Kinect2Obj_Distance, Kinect_Depth_Calib);
    
end

%%

[ tform_OOI, RMSE_OOI, PointCloud_OOI_transformed, PC_merged, PC_Final ] = PC_align_merge( PointCloud_OOI, nPoses );
