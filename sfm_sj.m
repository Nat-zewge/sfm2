clear
%close all

%% Use |imageDatastore| to get a list of all image file names in a
% directory.
imageDir = fullfile('images2');
imds = imageDatastore(imageDir);


%% load camera parameters
%K = [2389.33 0 1536; 0 2389.33 1152; 0 0 1]';
K = [2559.68 0 1536; 0 2559.68 1152; 0 0 1]';

cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Convert image
% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    Img{i} = readimage(imds, i);
%     Img{i} = undistortImage(Img{i}, cameraParams); 
    I{i} = rgb2gray(Img{i});
    
end

%% Undistort the first image.

% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 30;
roi = [border, border, size(I{1}, 2)- 2*border, size(I{1}, 1)- 2*border];

  ImagePoints   = detectSURFFeatures(I{1}, 'NumOctaves', 1,'ROI', roi);
%   ImagePoints = detectMinEigenFeatures(I{1}, 'MinQuality', 0.01);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.

    prevFeatures = extractFeatures(I{1} , ImagePoints, 'Upright', true,'SURF',64);


% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.

vSet = addView(vSet, 1, 'Points', ImagePoints, 'Orientation', ...
    eye(3, 'like', ImagePoints.Location), 'Location', ...
    zeros(1, 3, 'like', ImagePoints.Location));


%% add more views
 for i = 2:numel(images)
   
     
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I{i}, 'NumOctaves', 1,'ROI', roi);
      
%       currPoints = detectMinEigenFeatures(I{i}, 'MinQuality', 0.01);
      currFeatures = extractFeatures(I{i}, currPoints, 'Upright', true,'SURF',64);  
      indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .8, 'Unique',  true); %7ÀÏ¶§ 25¿¡Œ­ ÃÖŸÇ. 

    
    % Select matched points.
    matchedPoints1 = ImagePoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
%    figure; showMatchedFeatures(I{i-1},I{i},matchedPoints1,matchedPoints2);
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);
    
%      figure; showMatchedFeatures(I{i-1},I{i},matchedPoints1(inlierIdx),matchedPoints2(inlierIdx)); 

    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
    
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
        tracks = findTracks(vSet);
    
    
    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);

    
 %% Bundle Adjustment (selection)
        [ BA_xyz{1}, BA_caomPoses{1}, BA_reprojectionErrors{1}] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'MaxIterations' ,1000,'RelativeTolerance',1e-8,'AbsoluteTolerance' ,0.3,'FixedViewId', [1:i-1], ...
        'PointsUndistorted', false); 
    
         [BA_xyz{2}, BA_caomPoses{2}, BA_reprojectionErrors{2}] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'MaxIterations' ,1000,'RelativeTolerance',1e-8,'AbsoluteTolerance' ,0.3,'FixedViewId', [1:i-5], ...
        'PointsUndistorted', false);    
    
         if (sum(BA_reprojectionErrors{1})<=sum(BA_reprojectionErrors{2}))
            reprojectionErrors=BA_reprojectionErrors{1};
            xyzPoints=BA_xyz{1};
            camPoses=BA_caomPoses{1};
         else
            reprojectionErrors=BA_reprojectionErrors{2};
            xyzPoints=BA_xyz{2};
            camPoses=BA_caomPoses{2};             
         end
         
         S = sprintf('Aver.RepError is %.2f or %.2f',sum(BA_reprojectionErrors{1})/size(BA_reprojectionErrors{1},1),sum(BA_reprojectionErrors{2})/size(BA_reprojectionErrors{2},1));
         disp(S)
 

     S = sprintf('%d th : matches=%d / inliner = %d / #reporojection = %d /Aver.RepError= %.2f',i,size(indexPairs,1),numel(find(inlierIdx)),size(reprojectionErrors,1),sum(reprojectionErrors)/size(reprojectionErrors,1));
disp(S)
    
    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    ImagePoints   = currPoints;  
 end

 
%% display camera poses

    S = sprintf('Display camera poses');
    disp(S)
    
%     [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
%         tracks, camPoses, cameraParams, 'MaxIterations' ,300,'FixedViewId', 1, ...
%         'PointsUndistorted', false); 
%       S = sprintf('%d th : matches=%d / inliner = %d / #reporojection = %d /Aver.RepError= %.2f',i,size(indexPairs,1),numel(find(inlierIdx)),size(reprojectionErrors,1),sum(reprojectionErrors)/size(reprojectionErrors,1));
%       disp(S)
      
% Display camera poses 
camPoses = poses(vSet);
figure(3);
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on

hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-100, loc1(1)+100]);
ylim([loc1(2)-100, loc1(2)+100]);
zlim([loc1(3)-100, loc1(3)+100]);
camorbit(0, -30);

title('BA Camera Poses');


%% Dense Reconstruction  //==> sparse reconstruction
%     S = sprintf('Start dense reconstruction');
%     disp(S)
% %Read and undistort the first image
% %     I = undistortImage(images{1}, cameraParams);
% 
% % Detect corners in the first image.
% prevPoints = detectMinEigenFeatures(I{1}, 'MinQuality', 0.001);
% 
% % Create the point tracker object to track the points across views.
% tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);
% 
% % Initialize the point tracker.
% prevPoints = prevPoints.Location;
% initialize(tracker, prevPoints, I{1});
% 
% % Store the dense points in the view set.
% vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
% vSet = updateView(vSet, 1, 'Points', prevPoints);
% 
% % Track the points across all views.
%  for i = 2:numel(images)
%     % Read and undistort the current image.
% %     I = undistortImage(images{i}, cameraParams); 
%     % Track the points.
%     [currPoints, validIdx] = step(tracker, I{i});
% 
%     S = sprintf('%d th : validIdx=%d ',i,numel(find(validIdx)));
%     disp(S)
%     
%     % Clear the old matches between the points.
%     if i < numel(images)
%         vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
%     end
%     vSet = updateView(vSet, i, 'Points', currPoints);
%     
%     % Store the point matches in the view set.
%     matches = repmat((1:size(prevPoints, 1))', [1, 2]);
%     matches = matches(validIdx, :);        
%     vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
% end
% 
% % Find point tracks across all views.
% tracks = findTracks(vSet);
% 
% % Find point tracks across all views.
% camPoses = poses(vSet);
% 
% % Triangulate initial locations for the 3-D world points.
% xyzPoints = triangulateMultiview(tracks, camPoses,...
%     cameraParams);
% 
% % Refine the 3-D world points and camera poses.
% [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
%     xyzPoints, tracks, camPoses, cameraParams, 'MaxIterations' ,300,'FixedViewId', 1, ...
%     'PointsUndistorted', true);
% 
% %% Display Dense Reconstruction
%     S = sprintf('Display camera poses after dense reconstruction');
%     disp(S)
% % Display the refined camera poses.
% figure(3);
% plotCamera(camPoses, 'Size', 0.2);
% hold on
% 
% 
% % Exclude noisy 3-D world points.
% goodIdx = (reprojectionErrors < 5);
% 
% % Display the dense 3-D world points.
% pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
% grid on
% hold off
% 
% % Specify the viewing volume.
% loc1 = camPoses.Location{1};
% xlim([loc1(1)-100, loc1(1)+100]);
% ylim([loc1(2)-100, loc1(2)+100]);
% zlim([loc1(3)-100, loc1(3)+100]);
% camorbit(0, -80);
% 
% title('Dense Reconstruction');
% 
% % save_ply('south_building.ply',xyzPoints(goodIdx,:));
