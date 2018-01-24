left = [];
right = [];
clc
ls
addpath('averaging_quaternions')
cd /media/gtabor/log/'Calib files'/;
list = dir('*.jpg');
for k = 1:length(list)
   name = list(k).name;
   if startsWith(name,"left")
      left = [left;{name}];
   elseif startsWith(name,"right")
      right = [right;{name}];
   else
       "nothing";
   end
   
end



[imagePoints,boardSize] = detectCheckerboardPoints(left);

squareSize = 29; % millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

I = imread(char(left(1)));
imageSize = [size(I,1) size(I,2)];
cameraParamsLeft = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);

[imagePoints,boardSize] = detectCheckerboardPoints(right);
cameraParamsRight = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);



shared = [1,2,3,4,5,6];
rightRot = cameraParamsRight.RotationMatrices(:,:,shared);
leftRot = cameraParamsLeft.RotationMatrices(:,:,shared);

rightTrans = cameraParamsRight.TranslationVectors(shared,:);
leftTrans = cameraParamsLeft.TranslationVectors(shared,:);


rotm2eul(rightRot);
rotm2eul(leftRot);
%decide what transforms to treat as true

rots = [];
quats = [];
vectors = [];
for img = 1:6

    leftLoc = leftTrans(img,:);
    rightLoc = rightTrans(img,:);

    rightO = rightRot(:,:,img);
    leftO = leftRot(:,:,img);


    Left2Right = [[transpose(rightO), transpose(rightLoc)];[0,0,0,1]] * pinv([[transpose(leftO), transpose(leftLoc)];[0,0,0,1]]);
    rot = Left2Right(1:3,1:3);
    vect = Left2Right(1:3,4);
    rots = [rots;rot];
    vectors=[vectors,vect];
    quats = [quats;rotm2quat(transpose(rot))];
end
mean(vectors,2)
avg_quaternion_markley(quats)

rot = quat2rotm(transpose(avg_quaternion_markley(quats)))
LeftReal = [[transpose(rot),mean(vectors,2)];[0,0,0,1]]
RightReal = eye(4)



right = plotCamera('Location',transpose(RightReal(1:3,4)),'Orientation',transpose(RightReal(1:3,1:3)),'Size',20,'Label','right');
hold on
left = plotCamera('Location',transpose(LeftReal(1:3,4)),'Orientation',transpose(LeftReal(1:3,1:3)),'Size',20,'Label','left');
hold on


green = 0;
blue = 0;
for i = 1:size(cameraParamsRight.RotationMatrices,3)
    localRotation = cameraParamsRight.RotationMatrices(:,:,i);
    localTransloation = cameraParamsRight.TranslationVectors(i,:);
    local = [[transpose(localRotation), transpose(localTransloation)];[0,0,0,1]];
    for i = 1:size(cameraParamsRight.WorldPoints,1)
        x = cameraParamsRight.WorldPoints(i,1);
        y = cameraParamsRight.WorldPoints(i,2);
        point = RightReal*local *[x;y;0;1];
        plot3(point(1),point(2),point(3),'go');
    end
    green = green + 1;

end

for i = 1:size(cameraParamsLeft.RotationMatrices,3)
    localRotation = cameraParamsLeft.RotationMatrices(:,:,i);
    localTransloation = cameraParamsLeft.TranslationVectors(i,:);
    local = [[transpose(localRotation), transpose(localTransloation)];[0,0,0,1]];
    for i = 1:size(cameraParamsLeft.WorldPoints,1)
        x = cameraParamsLeft.WorldPoints(i,1);
        y = cameraParamsLeft.WorldPoints(i,2);
        point = LeftReal*local *[x;y;0;1];
        plot3(point(1),point(2),point(3),'b*');
    end
    blue = blue + 1;

end


%plot3(worldPoints(:,1),worldPoints(:,2),zeros(size(worldPoints, 1),1),'*');
%hold on


