% Use |imageDatastore| to get a list of all image file names in a
% directory.
imageDir = fullfile('images2');
imds = imageDatastore(imageDir);

% Display the images.
% figure
% montage(imds.Files, 'Size', [3, 2]);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = rgb2gray(I);
end

%title('Input Image Sequence');

% load(fullfile(imageDir, 'cameraParams.mat'));
K = [2559.68 0 1536; 0 2559.68 1152; 0 0 1]';
%K = [1698.873755 0.000000 971.7497705; 0.000000 1698.8796645 647.7488275; 0 0 1]';
%r = -[0.02,0,0];
cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Undistort the first image.
I = undistortImage(images{1}, cameraParams); 
%I = images{1}; 

% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 200;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints, 'Upright', true);

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;
    
% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
    eye(3, 'like', prevPoints.Location), 'Location', ...
    zeros(1, 3, 'like', prevPoints.Location));

%% add more views
for i = 2:numel(images)
    % Undistort the current image.
    I = undistortImage(images{i}, cameraParams);
    %I = images{i}; 

    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
    currFeatures = extractFeatures(I, currPoints, 'Upright', true);    
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', 0.9, 'Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(...
        matchedPoints1, matchedPoints2, cameraParams);
    
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
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, ...
        'PointsUndistorted', true,'MaxIterations',1500,'RelativeTolerance',1e-7,'AbsoluteTolerance',0.5);
   

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end

%% display camera poses

camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on



% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);


% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 30);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-20, loc1(1)+30]);
ylim([loc1(2)-20, loc1(2)+30]);
zlim([loc1(3)-20, loc1(3)+30]);
camorbit(0, -30);

title('Refined Camera Poses');

% % Read and undistort the first image
% I = undistortImage(images{1}, cameraParams); 
% Icolor = undistortImage(imagescolor{1}, cameraParams);  
% % Detect corners in the first image.
% prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);
% % Create the point tracker object to track the points across views.
% tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);
% % Initialize the point tracker.
% prevPoints = prevPoints.Location;
% initialize(tracker, prevPoints, I);
% % Store the dense points in the view set.
% vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
% vSet = updateView(vSet, 1, 'Points', prevPoints);
% % Track the points across all views.
% for i = 2:numel(images)
%     % Read and undistort the current image.
%     I = undistortImage(images{i}, cameraParams); 
%     
%     % Track the points.
%     [currPoints, validIdx] = step(tracker, I);
% % Clear the old matches between the points.
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
% % Find point tracks across all views.
% tracks = findTracks(vSet);
% % Find point tracks across all views.
% camPoses = poses(vSet);
% % Triangulate initial locations for the 3-D world points.
% xyzPoints = triangulateMultiview(tracks, camPoses,...
%     cameraParams);
% % Refine the 3-D world points and camera poses.
% [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
%         tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
%         'PointsUndistorted', true,'MaxIterations',1500,'RelativeTolerance',1e-8,'AbsoluteTolerance',0.5);
% % Display the refined camera poses.
% figure;
% plotCamera(camPoses, 'Size', 0.2);
% hold on
% % Exclude noisy 3-D world points.
% goodIdx = (reprojectionErrors < 5);
% aa = num2cell(tracks); 
% [R,~] = size(xyzPoints);
% trackpoints = zeros(R,2);
% for i = 1 : R
% trackpoints(i,1)=aa{1,i}.Points(1,1);
% trackpoints(i,2)=aa{1,i}.Points(1,2);      
% end
% numPixels = size(Icolor, 1) * size(Icolor, 2);
% allColors = reshape(Icolor, [numPixels, 3]);
% colorIdx = sub2ind([size(Icolor, 1), size(Icolor, 2)], round(trackpoints(:,2)),round(trackpoints(:,1)));
% color = allColors(colorIdx, :);
% xyzPoints = pointCloud(xyzPoints, 'Color', color);
% % Display the dense 3-D world points.
% pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
%     'MarkerSize', 45);
% grid on
% 
% hold off
% % Specify the viewing volume.
% loc1 = camPoses.Location{1};
% xlim([loc1(1)-20, loc1(1)+20]);
% ylim([loc1(2)-20, loc1(2)+20]);
% zlim([loc1(3)-20, loc1(3)+20]);
% camorbit(0, -30);
% title('Dense Reconstruction');


%% compute the average reprojection error after filtering for spurious points

% count = 0;
% for i = 1:size(reprojectionErrors)
% 
% if reprojectionErrors(i) < 5
%     filtered(i) = reprojectionErrors(i);
%     count = count + 1;
% else
%     filtered(i) = 0;
% 
% end
% 
% end
% filtered;
% avg_filtered = sum(filtered)/count;




