%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ME597 W23 HW2 Q1
% Using RANSAC for feature matching
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

clear;
%% Section 1: Read two images
Ia = im2gray(imread('Book_a.jpg'));
Ib = im2gray(imread('Book_b.jpg'));

%% Section 2: Detect SIFT features
points_a = detectSIFTFeatures(Ia);
points_b = detectSIFTFeatures(Ib);

% Extract 500 strongest features
[features_a,valid_points_a] = extractFeatures(Ia,points_a.selectStrongest(500));
[features_b,valid_points_b] = extractFeatures(Ib,points_b.selectStrongest(500));

% Try the initial matching (brute force matching)
% The resulting matched points have quite a few outliers
indexPairs = matchFeatures(features_a,features_b) ;
matchedPoints_a = valid_points_a(indexPairs(:,1));
matchedPoints_b = valid_points_b(indexPairs(:,2));

% These are the points from which you need to find inliers (as well as
% corresponding SE(2), i.e. the rotation matrix R and the translation vector t)
% pt1 are the matched feature points from Book_a.jpg with noise (with outliers)
% pt2 are the matched feature points from Book_b.jpg with noise (with outliers)
pt_a = matchedPoints_a.Location';
pt_b = matchedPoints_b.Location';

%% Section 3: Plot the initial matched features with outliers
% Figures show initial feature matching. Note that there are many outliers 
% (i.e. incorrect matches)
figure;ax = axes;
showMatchedFeatures(Ia,Ib,matchedPoints_a,matchedPoints_b,"montag",Parent=ax);
title(ax,"Candidate point matches", 'FontSize', 16);
legend(ax,"Initial matched points a","Initial matched points b", 'FontSize', 14);

%% Section 4: Start of RANSAC
% Initialization 
num_ftr = length(indexPairs);   % Total number of initial matching   
N_iter = 50;                    % The total number of iteration for random sampling
thresh_e = 20;                  % Threshold for determining inliers (in pixels)
f(N_iter).inlier = {};          % Initialization of the inlier set (defined as cell array)
f(N_iter).R = [];               % Initialization of the rotation matrix
f(N_iter).t = [];               % Initialization of the translation vector
Max_count = 0;                  % A variable for the most number of inliers

%% Section 5: Start of the RANSAC loop
% Start of the loop
for k = 1:N_iter
    % Randomly sample two feature indices.
    sampled_feature_indices = randsample(1:num_ftr, 2);    
    % End point of a vector sampled from "Book_a.jpg".
    p1a = pt_a(:,sampled_feature_indices(1));               
    % Start point of a vector sampled from "Book_a.jpg".
    p2a = pt_a(:,sampled_feature_indices(2));               
    % End point of a vector sampled from "Book_b.jpg".
    p1b = pt_b(:,sampled_feature_indices(1));               
    % Start point of a vector sampled from "Book_b.jpg".
    p2b = pt_b(:,sampled_feature_indices(2));               

    % Construct two vectors to compute SE(2) 
    va = p1a - p2a;     % A vector constructed by two randomly selected 
                        % sample features from the first image.
    vb = p1b - p2b;     % A vector constructed by the same feature points
                        % in the second image
    
    % You need to complete the code from here.
    % Replace "..." by your own code.
    % Use va and vb to find the rotation matrix R and the translation 
    % vector for SE(2).
    % First, find the angle between two vectors (by using atan2).
    th = atan2(va(2), va(1)) - atan2(vb(2), vb(1));    
    
    % Using the computed angle th, construct the rotation matrix R.
    R = [cos(th) -sin(th); sin(th) cos(th)];
    
    % TODO 5a: Compute the translation error vector for end points by first rotating
    % p1b by th and then subtracting it from p1a.
    e1 = p1a - R*p1b;    
    
    % TODO 5a: Compute the translation error vector for start points by first
    % rotating p2b by th and then subtracting it from p2a.
    e2 = p2a-R*p2b;        
    
    % TODO 5a: Take the mean of the above two error vectors as the translation 
    % vector t.
    t = (e1+e2)/2;
    
    % TODO 5b: Now, using R and t you computed above, transform all feature points
    % in Book_b.
    % If the transformed feature point is close enough to the corresponding
    % feature point in Book_a, then this feature point is an inlier.
    % Write your own code to find all the inliers.
    % The variable "inliers" defined below is a row vector containing
    % all the column numbers for inlier features. Namely, it is a subset of
    % [1, 2, 3, ..., num_ftr].

    inliers = [];
    for i = 1:num_ftr
        transformed_pt = R * pt_b(:, i) + t;
        err = norm(pt_a(:, i)-transformed_pt);
        if err < thresh_e
            inliers = [inliers, i];
        end
    end
    
    % Save the found R, t and inliers for the current iteration.
    f(k).R = R;
    f(k).t = t;
    f(k).inliers = inliers;
    
    % Compare the number of inliers at the current iteration with that from
    % the previous iteration. If the one for the current iteration is
    % bigger than the previous one, then make the current one as the
    % largest inlier size.
    if length(inliers) > Max_count
        Max_count = length(inliers);
        Max_ind = k;
    end
    
end

%% Section 6: Verify the feature matching
% Plot the matched features.
% In overlap
figure;
showMatchedFeatures(Ia,Ib,matchedPoints_a(f(Max_ind).inliers),matchedPoints_b(f(Max_ind).inliers));
title("Features Matched by RANSAC - Overlap", 'FontSize', 16);
legend("Matched points a","Matched points b", 'FontSize', 14);
% In montage
figure;ax = axes;
showMatchedFeatures(Ia,Ib,matchedPoints_a(f(Max_ind).inliers),matchedPoints_b(f(Max_ind).inliers),'montage',Parent=ax);
title(ax,"Features Matched by RANSAC - Montage", 'FontSize', 16);
legend(ax,"Matched points a","Matched points b", 'FontSize', 14);

