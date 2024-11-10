# Notes

## Motors
- Types of motor control: duty cycle, voltage (duty cycle + voltage comp), position/velocity duty cycle/voltage.

- Isupply = Istator * dutyCycle

- The stator current (Istator) is the current in the motor. The stator current goes up in order for the motor to produce torque (if say the motor's stuck, or the motor's accelerating) but this can be dangerous as with a lot of torque acceleration can go too high and with too much stator current the motor may produce too much heat. To prevent this, there are stator current limits in the motor.

- The supply current (Isupply) is the current taken in by the motor from the power supply. If this current becomes too high the power supply may be damaged. If for example the duty cycle was 100%, Istator would be equal to Isupply, and a safe Istator may be too high of a Isupply, which is why a supply current limit exists. Also stator current limits may sometimes not work, and supply current limits are a backup. 

## Faults
- Faults - Live behaviors on the device, some of which should be alerted (in the SwerveBase). Examples could be overheating device or hardware fault.

- StatusCode/RevLibError - The status of an action done on the device, whether it be setting the device's control, reading the velocity of the device, or changing a setting on the device. The status of setting the control/reading from the device doesn't have to be checked as if it fails (usually doesn't) it is done periodically and can work in the next iteration. The status of configuring a device should be checked as configuration only occurs once and should be retried until the status is successful.

## Cameras

### Calibration
Pw = Coordiante (X,Y,Z) of tracked point in world 3d space <br>
Pc = Coordinate (X,Y,Z) of tracked point in the camera's 3d space <br> 
Cw = Coordinate (X,Y,Z) of camera in world 3d space (also called extrinsics) <br>
(u, v) = Coordinate (X,Y) of tracked point as appeared on the camera image <br>
Camera instrinsics -> Internal camera parameters like distortion, focal point, etc 

Converting Pw to (u, v) is done by:
1) Pc = Apply Cw on Pw
2) (u, v) = Apply camera intrinsics on Pc

Camera intrinsics must be found before doing any 3d stuff through calibration:
A checkerboard containing many corners each with their own known coordinate Pw is used. The checkerboard
defines the world coordinate system, where the top-left corner is (0, 0, 0), and therefore the Z (depth) of
each corner coordinate would be 0 (the Z of Pw is 0). 1 unit along an axis in the world system is 1 unit along a chessboard square length. The checkerboard lays flat the whole calibration, so Z=0 doesn't change, and the camera moves around it. The camera takes many images from different orientations. During calibration the calibrator goes one by one through every image. For each image, the corners are detected, and the (u, v) for each corner is identified. On the starting corner a initially guessed intrinsics are used and Cw is solved for (using solvePnP). The calibrator will continue iterating through each corner, using Cw and the intrinsics to "re-project" the the known Pw of the corner onto the image plane, and find reprojection error which gets reduced as the intrinsics get refined. The intrinsics get even more refined as more corners from more images are reprojected, and in the end, the camera's overall intrinsics are returned from the calibration (along with Cw from every image). 

NOTE: The image can also move around a still camera, which is the same as a camera moving around a still image (and this is done in the case of FRC so you can calibrated with a still mounted camera).

NOTE: Charuco patterns are essentially just the checkerboard pattern but each "square" has a unique pattern making each corner uniquely identifiable. This is needed when no outer corners are seen of the calibration checkerboard and corners must be uniquely identified so their corresponding world cordinates can be found.

### Pose Estimation From a Single Tag (solvePnP) and Ambiguity
Pose estimation is kinda similar to camera calibration. Each apriltag has 4 corners with corresponding known 3d world-space coordinates (either in tag space or field space), and known image points. The camera intrisics are also known, so given all this known information the camera's extrinsics can be found. An algorithm called solvePnP will take in the world-space and image corner coordinates, and the camera intrinsics, to figure out the camera's extrinsics. Sometimes however, solvePnP may have 2 extrinsic results that have pretty good reprojection errors, and this is called ambiguity. Ambiguity can be measured by comparing these two reprojections errors, where ambiguity = better reprojection error / worse reprojection error. If ambiguity is around 1, it is unclear which of these extrinsics is the right one, while if the ambiguity is around 0, it becomes more obvious that the better reprojection error is significantly better than the worse reprojection error, and it becomes clear which extrinsic to use. When multiple tags are present, Photon Vision is able to use a different variation of solvePnP that takes in multiple 3d points (more than 4 in this case) and with many 3d points is able to return a single extrinsic much less ambigious since there isn't really another extrinsic with similar reprojection error (ambiguity is reduced but it still exists, meaning the extrinsic can jump around).

![Ambiguity Example](https://docs.wpilib.org/en/stable/_images/planar_ambiguity1.png)

### Noise and Standard Deviations
Sensors don't always return their consistent measurement due to noise, which is random measurements that occur and are incorrect / deviated from the correct consistent measurement. A pose measurement returned from photon vision can have noise due to two main factors, corner noise and pose solver noise. Corner noise is when the image coordinates of the corners (found by corner detection) have noise and that's caused by camera hardware, capture settings (like resolution and other pipeline tunings in pv), fov (makes corners closer together making it harder to detect), distance (same reason as fov, corners get closer together as distance increases), and etc. The other is pose solver noise, which is noise in the solved pose of the camera. Poor calibration of the camera creates pose solver noise and in this case there isn't exactly one single frequent consistent measurement with noise, it can be different (it's non-gaussian). 
Corner measurements look gaussian so the standard deviations are a good measurement of corner noise, but pose solver noise isn't gaussian but the standard deviations are a close enough measurement of the spread of data (noise) from the mean ("the correct measurement"). 

Although corner coordinate data can have noise, having more of that corner data will help reduce the ultimate pose noise. This means having multiple tags versus one tag will be less noisy. A heuristic model (not the exact answer, but reasonable enough) can be created to find the noise of some vision measurement (noise is standard deviation where the mean is the "correct measurement"). 


# Stuff to read about
- Robot states
- More on control loops (sys id)
- Pathplanner vs choreo