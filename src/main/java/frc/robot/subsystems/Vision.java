// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

/**
 * Vision subsystem — manages all PhotonVision cameras and fuses their pose
 * estimates into the swerve drive's {@link SwerveDrivePoseEstimator}.
 *
 * <p>Because this class extends {@link SubsystemBase}, the WPILib Command
 * Scheduler calls {@link #periodic()} automatically every 20 ms.  Simply
 * instantiating this class in {@code RobotContainer} is all that is needed;
 * no further wiring or manual calls are required.
 *
 * <p>To add a third (or fourth) camera, follow the three-step pattern marked
 * with "ADD CAMERA HERE" comments throughout this file.
 */
public class Vision extends SubsystemBase {

    // ── Cameras ───────────────────────────────────────────────────────────────
    // Camera names must match the names configured in PhotonVision's web UI.
    private final PhotonCamera frontCamera;
    private final PhotonCamera backCamera;
    // ADD CAMERA HERE (step 1 of 3): private final PhotonCamera leftCamera;

    // ── Per-camera pose estimators ────────────────────────────────────────────
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;
    // ADD CAMERA HERE (step 2 of 3): private final PhotonPoseEstimator leftPoseEstimator;

    // ── Per-camera Field2d widgets (SmartDashboard / Elastic visualisation) ──
    private final Field2d frontCameraField;
    private final Field2d backCameraField;
    // ADD CAMERA HERE (step 3 of 3): private final Field2d leftCameraField;

    // ── Reference to swerve subsystem ─────────────────────────────────────────
    private final SwerveSubsystem swerveSubsystem;

    // ── AprilTag field layout ─────────────────────────────────────────────────
    private final edu.wpi.first.apriltag.AprilTagFieldLayout aprilTagFieldLayout;

    // ── Trust parameters ──────────────────────────────────────────────────────
    // Standard deviations [x (m), y (m), θ (rad)].
    // Larger value → trust that axis less.  Rotation is intentionally high so
    // the gyro dominates heading estimation.
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS  = VecBuilder.fill(0.5, 0.5, 2.0);

    // Reject single-tag estimates with ambiguity above this threshold.
    // Lower = stricter. 0.2 is a reasonable starting point.
    private static final double MAX_POSE_AMBIGUITY = 0.2;

    // Reject estimates when the robot is further than this from every visible tag.
    private static final double MAX_VISION_DISTANCE_M = 5.0;

    // Runtime flag — set to false to pause all vision updates without destroying the subsystem.
    private boolean visionEnabled = true;


    /**
     * Constructs the Vision subsystem.
     *
     * @param swerveSubsystem The swerve drive subsystem whose internal pose
     *                        estimator will receive vision measurements.
     */
    public Vision(SwerveSubsystem swerveSubsystem)
    {
        this.swerveSubsystem = swerveSubsystem;

        // ── Camera initialisation ─────────────────────────────────────────────
        frontCamera = new PhotonCamera("front_camera");
        backCamera  = new PhotonCamera("back_camera");

        // ── AprilTag field layout ─────────────────────────────────────────────
        // Change the field constant to match the season if needed.
        try {
            aprilTagFieldLayout = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
                edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout: " + e.getMessage(), e);
        }

        // ── Pose estimators ───────────────────────────────────────────────────
        // Each Transform3d describes where the camera sits relative to the robot
        // centre (positive X = forward, positive Y = left, positive Z = up).
        // CHANGE THESE VALUES to match your physical robot!

        // Front camera: 30 cm forward, 25 cm up, pitched down 30°.
        frontPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(0.3, 0.0, 0.25),
                new edu.wpi.first.math.geometry.Rotation3d(0.0, Math.toRadians(-30), 0.0)
            )
        );

        // Back camera: 30 cm back, 25 cm up, pitched down 30°, yawed 180°.
        backPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new edu.wpi.first.math.geometry.Transform3d(
                new edu.wpi.first.math.geometry.Translation3d(-0.3, 0.0, 0.25),
                new edu.wpi.first.math.geometry.Rotation3d(0.0, Math.toRadians(-30), Math.toRadians(180))
            )
        );

        // ── SmartDashboard Field2d widgets ────────────────────────────────────
        frontCameraField = new Field2d();
        backCameraField  = new Field2d();
        SmartDashboard.putData("Vision/Front Camera Pose", frontCameraField);
        SmartDashboard.putData("Vision/Back Camera Pose",  backCameraField);

        SmartDashboard.putBoolean("Vision/Enabled", visionEnabled);
    }


    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic()
    {
        // Read the enabled flag from SmartDashboard so it can be toggled live.
        visionEnabled = SmartDashboard.getBoolean("Vision/Enabled", visionEnabled);

        if (!visionEnabled) {
            return;
        }

        // Process every camera every loop.
        updateVisionMeasurement(frontCamera, frontPoseEstimator, frontCameraField, "Front");
        updateVisionMeasurement(backCamera,  backPoseEstimator,  backCameraField,  "Back");
        // ADD CAMERA HERE: updateVisionMeasurement(leftCamera, leftPoseEstimator, leftCameraField, "Left");
    }


    // ── Core update logic ─────────────────────────────────────────────────────

    /**
     * Drains all unread results from {@code camera}, validates each estimate,
     * scales standard deviations by distance, and injects trusted measurements
     * into the swerve pose estimator.
     *
     * @param camera        Camera to read from.
     * @param poseEstimator Matching PhotonPoseEstimator.
     * @param field         Field2d widget to update for this camera.
     * @param cameraName    Short name used for SmartDashboard keys.
     */
    private void updateVisionMeasurement(
            PhotonCamera camera,
            PhotonPoseEstimator poseEstimator,
            Field2d field,
            String cameraName)
    {
        var results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {

            // ── Bail early if no targets visible ─────────────────────────────
            if (!result.hasTargets()) {
                SmartDashboard.putBoolean("Vision/" + cameraName + "/HasTargets", false);
                continue;
            }

            SmartDashboard.putBoolean("Vision/" + cameraName + "/HasTargets", true);
            SmartDashboard.putNumber("Vision/" + cameraName + "/TargetCount",
                                     result.getTargets().size());

            // ── Estimate pose ─────────────────────────────────────────────────
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
            if (estimatedPose.isEmpty()) {
                SmartDashboard.putBoolean("Vision/" + cameraName + "/ValidPose", false);
                continue;
            }

            EstimatedRobotPose robotPose    = estimatedPose.get();
            Pose2d             estimatedPose2d = robotPose.estimatedPose.toPose2d();

            // Update the per-camera field widget
            field.setRobotPose(estimatedPose2d);

            // ── Quality checks ────────────────────────────────────────────────
            double poseAmbiguity       = result.getBestTarget().getPoseAmbiguity();
            double closestTagDistanceM = Double.MAX_VALUE;

            for (var target : result.getTargets()) {
                var tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    double dist = tagPose.get().toPose2d().getTranslation()
                                         .getDistance(estimatedPose2d.getTranslation());
                    closestTagDistanceM = Math.min(closestTagDistanceM, dist);
                }
            }

            boolean shouldTrust = true;
            String  rejectReason = "";

            if (poseAmbiguity > MAX_POSE_AMBIGUITY) {
                shouldTrust  = false;
                rejectReason = String.format("High ambiguity (%.3f > %.2f)",
                                             poseAmbiguity, MAX_POSE_AMBIGUITY);
            } else if (closestTagDistanceM > MAX_VISION_DISTANCE_M) {
                shouldTrust  = false;
                rejectReason = String.format("Tag too far (%.2f m > %.1f m)",
                                             closestTagDistanceM, MAX_VISION_DISTANCE_M);
            }

            // ── Telemetry ─────────────────────────────────────────────────────
            SmartDashboard.putBoolean("Vision/" + cameraName + "/ValidPose",    true);
            SmartDashboard.putBoolean("Vision/" + cameraName + "/Trusted",      shouldTrust);
            SmartDashboard.putString ("Vision/" + cameraName + "/RejectReason", rejectReason);
            SmartDashboard.putNumber ("Vision/" + cameraName + "/Ambiguity",    poseAmbiguity);
            SmartDashboard.putNumber ("Vision/" + cameraName + "/TagDistanceM", closestTagDistanceM);
            SmartDashboard.putNumber ("Vision/" + cameraName + "/Timestamp",    robotPose.timestampSeconds);

            if (!shouldTrust) {
                continue;
            }

            // ── Standard deviation scaling ────────────────────────────────────
            // Start from multi-tag (tight) or single-tag (loose) base values,
            // then scale up linearly with distance so far-away estimates are
            // trusted less.
            Matrix<N3, N1> baseStdDevs = (result.getTargets().size() > 1)
                    ? MULTI_TAG_STD_DEVS
                    : SINGLE_TAG_STD_DEVS;

            String stdDevType = (result.getTargets().size() > 1) ? "Multi-tag" : "Single-tag";
            SmartDashboard.putString("Vision/" + cameraName + "/StdDevType", stdDevType);

            double distanceScale = 1.0 + (closestTagDistanceM / MAX_VISION_DISTANCE_M);
            Matrix<N3, N1> scaledStdDevs = VecBuilder.fill(
                baseStdDevs.get(0, 0) * distanceScale,
                baseStdDevs.get(1, 0) * distanceScale,
                baseStdDevs.get(2, 0) * distanceScale
            );

            SmartDashboard.putNumber("Vision/" + cameraName + "/StdDev_X",     scaledStdDevs.get(0, 0));
            SmartDashboard.putNumber("Vision/" + cameraName + "/StdDev_Y",     scaledStdDevs.get(1, 0));
            SmartDashboard.putNumber("Vision/" + cameraName + "/StdDev_Theta", scaledStdDevs.get(2, 0));

            // ── Fuse into swerve pose estimator ───────────────────────────────
            swerveSubsystem.getSwerveDrive().addVisionMeasurement(
                estimatedPose2d,
                robotPose.timestampSeconds,
                scaledStdDevs
            );
        }
    }


    // ── Public API ────────────────────────────────────────────────────────────

    /** @return Latest unread results from the front camera. */
    public java.util.List<PhotonPipelineResult> getFrontCameraResults() {
        return frontCamera.getAllUnreadResults();
    }

    /** @return Latest unread results from the back camera. */
    public java.util.List<PhotonPipelineResult> getBackCameraResults() {
        return backCamera.getAllUnreadResults();
    }

    /**
     * Returns the most recent valid pose estimate from the front camera, if any.
     * Primarily for diagnostics / unit tests.
     */
    public Optional<Pose2d> getFrontCameraPose() {
        return getLatestPose(frontCamera, frontPoseEstimator);
    }

    /**
     * Returns the most recent valid pose estimate from the back camera, if any.
     * Primarily for diagnostics / unit tests.
     */
    public Optional<Pose2d> getBackCameraPose() {
        return getLatestPose(backCamera, backPoseEstimator);
    }

    /** Enable or disable all vision updates at runtime. */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        SmartDashboard.putBoolean("Vision/Enabled", enabled);
    }

    /** @return {@code true} if vision updates are currently enabled. */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }


    // ── Private helpers ───────────────────────────────────────────────────────

    private Optional<Pose2d> getLatestPose(PhotonCamera camera, PhotonPoseEstimator estimator)
    {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return Optional.empty();

        var result = results.get(results.size() - 1);
        if (!result.hasTargets()) return Optional.empty();

        return estimator.update(result)
                        .map(est -> est.estimatedPose.toPose2d());
    }
}