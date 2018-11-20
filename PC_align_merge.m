function [ tform_OOI, RMSE_OOI, PointCloud_OOI_transformed, PC_merged, PC_Final ] = PC_align_merge( PointCloud_OOI, nPoses )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Mid_Frm_Count = ceil(nPoses/2);

i = 1;

figure;
pcshowpair(PointCloud_OOI{i},PointCloud_OOI{i+1},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

[tform_OOI{i},PointCloud_OOI_transformed{i},RMSE_OOI{i}] = pcregrigid(PointCloud_OOI{i},PointCloud_OOI{i+1},'Metric','pointToPlane','Extrapolate',true,'MaxIterations',60,'Tolerance',[0.00001,0.00001],'Verbose',true);

figure;
pcshowpair(PointCloud_OOI{i+1},PointCloud_OOI_transformed{i},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

PC_merged{i} = pcmerge(PointCloud_OOI{i+1},PointCloud_OOI_transformed{i},0.00001);

figure;
pcshow(PC_merged{i},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

for i = 2 : Mid_Frm_Count-1
    
    figure;
    pcshowpair(PC_merged{i-1},PointCloud_OOI{i+1},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    
    [tform_OOI{i},PointCloud_OOI_transformed{i},RMSE_OOI{i}] = pcregrigid(PC_merged{i-1},PointCloud_OOI{i+1},'Metric','pointToPlane','Extrapolate',true,'MaxIterations',60,'Tolerance',[0.00001,0.00001],'Verbose',true);
    
    figure;
    pcshowpair(PointCloud_OOI{i+1},PointCloud_OOI_transformed{i},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    
    PC_merged{i} = pcmerge(PointCloud_OOI{i+1},PointCloud_OOI_transformed{i},0.00001);
    
    figure;
    pcshow(PC_merged{i},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    
end

i = nPoses;

figure;
pcshowpair(PointCloud_OOI{i},PointCloud_OOI{i-1},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

[tform_OOI{i-1},PointCloud_OOI_transformed{i-1},RMSE_OOI{i-1}] = pcregrigid(PointCloud_OOI{i},PointCloud_OOI{i-1},'Metric','pointToPlane','Extrapolate',true,'MaxIterations',60,'Tolerance',[0.00001,0.00001],'Verbose',true);

figure;
pcshowpair(PointCloud_OOI{i-1},PointCloud_OOI_transformed{i-1},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

PC_merged{i-1} = pcmerge(PointCloud_OOI{i-1},PointCloud_OOI_transformed{i-1},0.00001);

figure;
pcshow(PC_merged{i-1},'VerticalAxis','Y','VerticalAxisDir','Down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

for i = nPoses-1 : -1 : Mid_Frm_Count+1
    
    figure;
    pcshowpair(PointCloud_OOI{i-1},PC_merged{i},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    [tform_OOI{i-1},PointCloud_OOI_transformed{i-1},RMSE_OOI{i-1}] = pcregrigid(PC_merged{i},PointCloud_OOI{i-1},'Metric','pointToPlane','Extrapolate',true,'MaxIterations',60,'Tolerance',[0.00001,0.00001],'Verbose',true);

    figure;
    pcshowpair(PointCloud_OOI{i-1},PointCloud_OOI_transformed{i-1},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    PC_merged{i-1} = pcmerge(PointCloud_OOI{i-1},PointCloud_OOI_transformed{i-1},0.00001);

    figure;
    pcshow(PC_merged{i-1},'VerticalAxis','Y','VerticalAxisDir','Down');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

end

PC_Final = pcmerge(PC_merged{Mix_Frm_Count-1},PC_merged{Mix_Frm_Count},0.000001);

end