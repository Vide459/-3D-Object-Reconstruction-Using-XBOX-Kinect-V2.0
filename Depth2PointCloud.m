function [ PC_Out ] = Depth2PointCloud( DepthIMG, Min_depth, DepthCalib )
% This function is used to convert a XBOX Kinect V2.0 depthmap into a
%corresponding point cloud based on the Focal Lengths along the X and Y
%direction (Fx, Fy), and the Principal point(Cx, Cy). It also takes into
%account the radial and tangential distortions(K1, K2, K3 & P1, P2) in the
%inital  Kinect depthmap and takes measures to counter act them.

Fx = DepthCalib.FocalLengthX;
Fy = DepthCalib.FocalLengthY;

Cx = DepthCalib.PrincipalPointX;
Cy = DepthCalib.PrincipalPointY;

K1 = DepthCalib.RadialDistortionSecondOrder;
K2 = DepthCalib.RadialDistortionFourthOrder;
K3 = DepthCalib.RadialDistortionSixthOrder;

Kinect_Depth_Intrinsic_par = [Fx 0 0; 0 Fy 0; Cx Cy 1];

Kinect_Depth_RadialDistortion = [K1 K2 K3];

Kinect_Depth_Parameters = cameraParameters('IntrinsicMatrix',Kinect_Depth_Intrinsic_par,'RadialDistortion',Kinect_Depth_RadialDistortion);

[DepthIMG_Undistorted, ~] = undistortImage(DepthIMG, Kinect_Depth_Parameters);

figure;
imshow(DepthIMG_Undistorted,[]);

[r,c] = size(DepthIMG_Undistorted);

DepthIMG_Undistorted = double(DepthIMG_Undistorted);

k = 0;

for i = 1 : r
    for j = 1 : c
        k = k + 1;
        if DepthIMG_Undistorted(i,j) >= Min_depth
%             X(k) = ((j - Cx) / Fx) * DepthIMG_Undistorted(i,j);
%             Y(k) = ((i - Cy) / Fy) * DepthIMG_Undistorted(i,j);
%             Z(k) = DepthIMG_Undistorted(i,j);
            X(i,j) = ((j - Cx) / Fx) * DepthIMG_Undistorted(i,j);
            Y(i,j) = ((i - Cy) / Fy) * DepthIMG_Undistorted(i,j);
            Z(i,j) = DepthIMG_Undistorted(i,j) - Min_depth; 
        end
    end
end

XYZ_pts(:,:,1) = -X./1000;
XYZ_pts(:,:,2) = Y./1000;
XYZ_pts(:,:,3) = Z./1000;

PC_Out = pcdenoise(pointCloud(XYZ_pts));

figure;
pcshow(PC_Out,'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

end