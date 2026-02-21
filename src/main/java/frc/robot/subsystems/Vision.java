// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {
    
    // Cameras - add or remove cameras as needed
    private final PhotonCamera frontCamera;
    private final PhotonCamera backCamera;
    // Add more cameras here as needed
    // private final PhotonCamera leftCamera;
    // private final PhotonCamera rightCamera;
    
    // Pose estimators for each camera (for troubleshooting)
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;
    // Add more pose estimators here
    // private final PhotonPoseEstimator leftPoseEstimator;
    // private final PhotonPoseEstimator rightPoseEstimator;
    
    // Field2d objects for visualizing each camera's pose estimate
    private final Field2d frontCameraField;
    private final Field2d backCameraField;
    // Add more field objects for additional cameras
    
    // Reference to swerve subsystem for pose updates
    private final SwerveSubsystem swerveSubsystem;
    
    // AprilTag field layout
    private final edu.wpi.first.apriltag.AprilTagFieldLayout aprilTagFieldLayout;
    
    // Standard deviations for pose estimation
    // Higher values = trust vision less, lower values = trust vision more
    // [x, y, rotation] - we set rotation very high to trust gyro more
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 2.0);
    
    // Maximum acceptable pose ambiguity (lower is better)
    private static final double MAX_POSE_AMBIGUITY = 0.2;
    
    // Maximum distance to trust vision measurements (in meters)
    private static final double MAX_VISION_DISTANCE = 5.0;
    
    public Vision(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Initialize cameras with their network table names
        frontCamera = new PhotonCamera("front_camera");
        backCamera = new PhotonCamera("back_camera");
        // Add more cameras:
        // leftCamera = new PhotonCamera("left_camera");
        // rightCamera = new PhotonCamera("right_camera");
        
        // Load AprilTag field layout (2026 Reefscape field)
        // Change this to match your game year
        try {
            aprilTagFieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
                edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded
            );
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
        
        // Initialize pose estimators for each camera
        // You need to set the robot-to-camera transform for each camera
        // This is the physical position of the camera relative to the robot center
        
        // Front camera transform - CHANGE THESE VALUES TO MATCH YOUR ROBOT
        // Example: camera is 0.3m forward, 0m left/right, 0.25m up, pitched down 30 degrees
        frontPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(0.3, 0.0, 0.25),
                new edu.wpi.first.math.geometry.Rotation3d(0, Math.toRadians(-30), 0)
            )
        );
        
        // Back camera transform - CHANGE THESE VALUES TO MATCH YOUR ROBOT
        // Example: camera is 0.3m backward, 0m left/right, 0.25m up, pitched down 30 degrees, rotated 180
        backPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(-0.3, 0.0, 0.25),
                new edu.wpi.first.math.geometry.Rotation3d(0, Math.toRadians(-30), Math.toRadians(180))
            )
        );
        
        // Add more cameras:
        /*
        leftPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(0.0, 0.3, 0.25),
                new edu.wpi.first.math.geometry.Rotation3d(0, Math.toRadians(-30), Math.toRadians(90))
            )
        );
        */
        
        // Initialize Field2d objects for visualization
        frontCameraField = new Field2d();
        backCameraField = new Field2d();
        // Add more:
        // leftCameraField = new Field2d();
        
        SmartDashboard.putData("Front Camera Pose", frontCameraField);
        SmartDashboard.putData("Back Camera Pose", backCameraField);
        // SmartDashboard.putData("Left Camera Pose", leftCameraField);
    }
    
    @Override
    public void periodic() {
        // Update pose estimates from each camera
        updateVisionMeasurement(frontCamera, frontPoseEstimator, frontCameraField, "Front");
        updateVisionMeasurement(backCamera, backPoseEstimator, backCameraField, "Back");
        // Add more cameras:
        // updateVisionMeasurement(leftCamera, leftPoseEstimator, leftCameraField, "Left");
    }
    
    /**
     * Updates the global pose estimate using vision measurements from a specific camera
     * 
     * @param camera The PhotonCamera to get results from
     * @param poseEstimator The PhotonPoseEstimator for this camera
     * @param field The Field2d object for visualization
     * @param cameraName Name of the camera for logging
     */
    private void updateVisionMeasurement(
            PhotonCamera camera, 
            PhotonPoseEstimator poseEstimator,
            Field2d field,
            String cameraName) {
        
        // Get all unread results from the camera
        var results = camera.getAllUnreadResults();
        
        // Process each result
        for (PhotonPipelineResult result : results) {
            // Check if we have any targets
            if (!result.hasTargets()) {
                SmartDashboard.putBoolean(cameraName + " Has Targets", false);
                continue;
            }
            
            SmartDashboard.putBoolean(cameraName + " Has Targets", true);
            SmartDashboard.putNumber(cameraName + " Target Count", result.getTargets().size());
            
            // Get estimated pose from this camera
            // The pose estimator uses the strategy set in its constructor
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            
            if (estimatedPose.isEmpty()) {
                SmartDashboard.putBoolean(cameraName + " Valid Pose", false);
                continue;
            }
            
            EstimatedRobotPose robotPose = estimatedPose.get();
            Pose2d estimatedPose2d = robotPose.estimatedPose.toPose2d();
            
            // Update field visualization for this camera
            field.setRobotPose(estimatedPose2d);
            
            // Get the best target to check ambiguity
            var bestTarget = result.getBestTarget();
            double poseAmbiguity = bestTarget.getPoseAmbiguity();
            
            // Calculate distance to closest tag
            double closestTagDistance = Double.MAX_VALUE;
            for (var target : result.getTargets()) {
                var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    double distance = tagPose.get().toPose2d().getTranslation()
                        .getDistance(estimatedPose2d.getTranslation());
                    closestTagDistance = Math.min(closestTagDistance, distance);
                }
            }
            
            // Determine if we should trust this measurement
            boolean shouldTrust = true;
            String rejectReason = "";
            
            if (poseAmbiguity > MAX_POSE_AMBIGUITY) {
                shouldTrust = false;
                rejectReason = "High ambiguity: " + String.format("%.3f", poseAmbiguity);
            } else if (closestTagDistance > MAX_VISION_DISTANCE) {
                shouldTrust = false;
                rejectReason = "Too far from tags: " + String.format("%.2fm", closestTagDistance);
            }
            
            // Log debug information
            SmartDashboard.putBoolean(cameraName + " Valid Pose", true);
            SmartDashboard.putBoolean(cameraName + " Trusted", shouldTrust);
            SmartDashboard.putString(cameraName + " Reject Reason", rejectReason);
            SmartDashboard.putNumber(cameraName + " Ambiguity", poseAmbiguity);
            SmartDashboard.putNumber(cameraName + " Distance to Tag", closestTagDistance);
            SmartDashboard.putNumber(cameraName + " Timestamp", robotPose.timestampSeconds);
            
            if (!shouldTrust) {
                continue;
            }
            
            // Determine standard deviations based on number of tags and distance
            Matrix<N3, N1> stdDevs;
            if (result.getTargets().size() > 1) {
                // Multiple tags - more confident
                stdDevs = MULTI_TAG_STD_DEVS;
                SmartDashboard.putString(cameraName + " Std Dev Type", "Multi-tag");
            } else {
                // Single tag - less confident
                stdDevs = SINGLE_TAG_STD_DEVS;
                SmartDashboard.putString(cameraName + " Std Dev Type", "Single-tag");
            }
            
            // Scale standard deviations based on distance
            // Further away = less trust
            double distanceScaling = 1.0 + (closestTagDistance / MAX_VISION_DISTANCE);
            Matrix<N3, N1> scaledStdDevs = VecBuilder.fill(
                stdDevs.get(0, 0) * distanceScaling,
                stdDevs.get(1, 0) * distanceScaling,
                stdDevs.get(2, 0) * distanceScaling
            );
            
            // Add vision measurement to the swerve drive's pose estimator
            // The high rotation standard deviation means we trust the gyro much more than vision for rotation
            swerveSubsystem.getSwerveDrive().addVisionMeasurement(
                estimatedPose2d,
                robotPose.timestampSeconds,
                scaledStdDevs
            );
            
            SmartDashboard.putNumber(cameraName + " X Std Dev", scaledStdDevs.get(0, 0));
            SmartDashboard.putNumber(cameraName + " Y Std Dev", scaledStdDevs.get(1, 0));
            SmartDashboard.putNumber(cameraName + " Theta Std Dev", scaledStdDevs.get(2, 0));
        }
    }
    
    /**
     * Gets the latest results from the front camera
     */
    public java.util.List<PhotonPipelineResult> getFrontCameraResults() {
        return frontCamera.getAllUnreadResults();
    }
    
    /**
     * Gets the latest results from the back camera
     */
    public java.util.List<PhotonPipelineResult> getBackCameraResults() {
        return backCamera.getAllUnreadResults();
    }
    
    /**
     * Get the estimated pose from a specific camera (for troubleshooting)
     */
    public Optional<Pose2d> getFrontCameraPose() {
        var results = frontCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        var result = results.get(results.size() - 1); // Get the most recent
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        var estimate = frontPoseEstimator.update(result);
        return estimate.map(pose -> pose.estimatedPose.toPose2d());
    }
    
    /**
     * Get the estimated pose from the back camera (for troubleshooting)
     */
    public Optional<Pose2d> getBackCameraPose() {
        var results = backCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        var result = results.get(results.size() - 1); // Get the most recent
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        var estimate = backPoseEstimator.update(result);
        return estimate.map(pose -> pose.estimatedPose.toPose2d());
    }
    
    /**
     * Disable all vision updates (useful for testing)
     */
    private boolean visionEnabled = true;
    
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }
    
    public boolean isVisionEnabled() {
        return visionEnabled;
    }
}