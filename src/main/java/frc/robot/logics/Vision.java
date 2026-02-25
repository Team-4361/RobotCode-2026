package frc.robot.logics;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

public class Vision {

    private final SwerveSubsystem drivebase;
    // -------------------------------------------------------------------------
    // TUNING CONSTANTS - Adjust these for your robot
    // -------------------------------------------------------------------------

    // Maximum distance (meters) from a tag before we reject the measurement.
    // Lower = stricter. Start around 4-5m and tune.
    private static final double MAX_TAG_DISTANCE_METERS = 4.5;

    // If the vision pose is more than this far (meters) from odometry, reject it.
    // Prevents wild jumps. ~1.0m is a good starting point.
    private static final double MAX_VISION_ODO_DIFF_METERS = 1.0;

    // Pose ambiguity threshold (0 = perfect, 0.2+ = bad). Only matters for single tags.
    private static final double MAX_POSE_AMBIGUITY = 0.15;

    // Standard deviations for vision measurements:
    //   [x_meters, y_meters, rotation_radians]
    //
    // HIGH rotation std dev = we barely trust the vision rotation (rely on gyro instead).
    // Lower x/y std dev = we trust the position more.
    //
    // Multi-tag is more reliable so we trust it more (lower std devs).
    private static final Matrix<N3, N1> STD_MULTI_TAG  = VecBuilder.fill(0.5,  0.5,  9999.0);
    private static final Matrix<N3, N1> STD_SINGLE_TAG = VecBuilder.fill(0.9,  0.9,  9999.0);

    // Set to true ONLY if you want vision to also correct the gyro heading.
    // Generally leave false - gyro is much more accurate for rotation.
    public boolean updateHeadingWithVision = false;

    // -------------------------------------------------------------------------
    // CAMERA DEFINITIONS - Add or remove cameras here
    // -------------------------------------------------------------------------

    // Camera names must match exactly what's configured in PhotonVision
    PhotonCamera frontCamera = new PhotonCamera("frontCam");
    PhotonCamera backCamera  = new PhotonCamera("backCam");

    // Transforms: robot-center → camera lens
    // Positive X = forward, positive Y = left, positive Z = up
    Transform3d frontCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.0),   // Forward from center
            Units.inchesToMeters(0.0),    // Left/right (0 = centered)
            Units.inchesToMeters(22.0)    // Height above ground
        ),
        new Rotation3d(
            0,                             // Roll  (0 = level)
            Units.degreesToRadians(-20),   // Pitch (negative = angled down toward tags)
            Units.degreesToRadians(0)      // Yaw   (0 = facing forward)
        )
    );

    Transform3d backCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-13.0),  // Negative X = rear of robot
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(22.0)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(180)   // Yaw 180 = facing backward
        )
    );

    // -------------------------------------------------------------------------
    // POSE ESTIMATORS
    // -------------------------------------------------------------------------

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    PhotonPoseEstimator frontEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCameraTransform
    );

    PhotonPoseEstimator backEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        backCameraTransform
    );

    // -------------------------------------------------------------------------
    // DEBUG FIELDS - Each camera gets its own Field2d so you can compare them
    // on Shuffleboard/Glass before they are fused into odometry
    // -------------------------------------------------------------------------

    Field2d frontCameraDebugField = new Field2d();
    Field2d backCameraDebugField  = new Field2d();
    Field2d fusedOdometryField    = new Field2d(); // What YAGSL actually thinks

    // Timestamp of the last accepted AprilTag measurement
    public double timeAtLastSeen = 0.0;

    // -------------------------------------------------------------------------
    // CONSTRUCTOR
    // -------------------------------------------------------------------------

    public Vision(SwerveSubsystem drivebaseIn) {
        this.drivebase = drivebaseIn;
        // Register debug fields on SmartDashboard
        SmartDashboard.putData("Vision/FrontCam Raw Pose",  frontCameraDebugField);
        SmartDashboard.putData("Vision/BackCam Raw Pose",   backCameraDebugField);
        SmartDashboard.putData("Vision/Fused Odometry Pose", fusedOdometryField);

        SmartDashboard.putBoolean("Vision/Update Heading With Vision", updateHeadingWithVision);
    }

    // -------------------------------------------------------------------------
    // MAIN UPDATE - Call this from Robot.periodic() or a subsystem periodic
    // -------------------------------------------------------------------------

    public void updateVision() {
        // Process each camera independently
        processCamera(frontCamera, frontEstimator, frontCameraDebugField, "Front");
        processCamera(backCamera,  backEstimator,  backCameraDebugField,  "Back");

        // Always update the fused odometry debug field so you can see the result
        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().getPose());

        // Global dashboard values
        SmartDashboard.putNumber("Vision/Time Since Last Tag",   Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",     (Timer.getFPGATimestamp() - timeAtLastSeen) < 0.5);
    }

    // -------------------------------------------------------------------------
    // CAMERA PROCESSING
    // -------------------------------------------------------------------------

    /**
     * Processes all unread results from a single camera and feeds accepted
     * measurements into YAGSL's built-in pose estimator via addVisionMeasurement().
     *
     * This is the method that actually updates the global robot pose that
     * PathPlanner and all other subsystems use.
     *
     * @param camera         The PhotonCamera to read from
     * @param estimator      The corresponding PhotonPoseEstimator
     * @param debugField     Field2d for raw (pre-fusion) debug visualization
     * @param cameraLabel    Short name used in SmartDashboard keys (e.g. "Front")
     */
    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                Field2d debugField, String cameraLabel) {

        String prefix = "Vision/" + cameraLabel + "/";

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {

            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> maybeEstimate = estimator.update(result);
            if (maybeEstimate.isEmpty()) continue;

            EstimatedRobotPose estimate = maybeEstimate.get();
            Pose2d visionPose = estimate.estimatedPose.toPose2d();
            Pose2d odoPose    = drivebase.getSwerveDrive().getPose();

            // --- Always show the raw camera pose for debugging ---
            debugField.setRobotPose(visionPose);
            SmartDashboard.putNumber(prefix + "Raw X",        visionPose.getX());
            SmartDashboard.putNumber(prefix + "Raw Y",        visionPose.getY());
            SmartDashboard.putNumber(prefix + "Raw Rotation", visionPose.getRotation().getDegrees());

            // --- Gather rejection criteria ---
            double tagDist         = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            double poseAmbiguity   = result.getBestTarget().getPoseAmbiguity();
            int    targetCount     = result.getTargets().size();
            double poseDifference  = visionPose.getTranslation().getDistance(odoPose.getTranslation());
            boolean isMultiTag     = targetCount > 1;

            // Log all metrics so you can see why a measurement was rejected
            SmartDashboard.putNumber(prefix + "Tag Distance (m)",    tagDist);
            SmartDashboard.putNumber(prefix + "Pose Ambiguity",      poseAmbiguity);
            SmartDashboard.putNumber(prefix + "Target Count",        targetCount);
            SmartDashboard.putNumber(prefix + "Vs Odo Diff (m)",     poseDifference);
            SmartDashboard.putBoolean(prefix + "Is Multi-Tag",       isMultiTag);

            // --- Acceptance logic ---
            //
            // Multi-tag: only check distance and odo difference (ambiguity is less relevant)
            // Single tag: also check ambiguity to throw out bad single-tag solves
            boolean distOK       = tagDist < MAX_TAG_DISTANCE_METERS;
            boolean diffOK       = poseDifference < MAX_VISION_ODO_DIFF_METERS;
            boolean ambiguityOK  = isMultiTag || (poseAmbiguity < MAX_POSE_AMBIGUITY && poseAmbiguity >= 0);

            boolean accepted = distOK && diffOK && ambiguityOK;

            SmartDashboard.putBoolean(prefix + "Distance OK",   distOK);
            SmartDashboard.putBoolean(prefix + "Diff OK",       diffOK);
            SmartDashboard.putBoolean(prefix + "Ambiguity OK",  ambiguityOK);
            SmartDashboard.putBoolean(prefix + "ACCEPTED",      accepted);

            if (!accepted) {
                SmartDashboard.putString(prefix + "Reject Reason",
                    !distOK ? "Tag too far" : !diffOK ? "Jump too large" : "High ambiguity");
                continue;
            }

            // --- Build the pose to submit ---
            // We keep the gyro's rotation and only correct X/Y from vision,
            // unless updateHeadingWithVision is explicitly enabled.
            Pose2d poseToSubmit;
            if (updateHeadingWithVision) {
                poseToSubmit = visionPose; // trust vision rotation too
            } else {
                poseToSubmit = new Pose2d(
                    visionPose.getTranslation(),
                    odoPose.getRotation()  // keep current gyro heading
                );
            }

            // --- Choose standard deviations ---
            // High rotation std dev (9999) = don't let vision touch the gyro heading.
            // Lower x/y values for multi-tag since the solution is much more reliable.
            Matrix<N3, N1> stdDevs;
            if (updateHeadingWithVision) {
                stdDevs = isMultiTag
                    ? VecBuilder.fill(0.4, 0.4, 0.6)
                    : VecBuilder.fill(0.8, 0.8, 1.2);
            } else {
                stdDevs = isMultiTag ? STD_MULTI_TAG : STD_SINGLE_TAG;
            }

            // Scale trust by distance - farther tag = less trust
            // Multiply std devs up as tag gets farther away
            double distanceScaleFactor = 1.0 + (tagDist / MAX_TAG_DISTANCE_METERS);
            Matrix<N3, N1> scaledStdDevs = VecBuilder.fill(
                stdDevs.get(0, 0) * distanceScaleFactor,
                stdDevs.get(1, 0) * distanceScaleFactor,
                stdDevs.get(2, 0) // Don't scale rotation (it's already huge or tuned)
            );

            // ---------------------------------------------------------------
            // THIS IS THE KEY CALL - feeds vision into YAGSL's pose estimator.
            // This is the SAME pose estimator PathPlanner reads from, so this
            // correction will help PathPlanner autonomous stay on path!
            // ---------------------------------------------------------------
            drivebase.getSwerveDrive().addVisionMeasurement(
                poseToSubmit,
                estimate.timestampSeconds,
                scaledStdDevs
            );

            timeAtLastSeen = Timer.getFPGATimestamp();

            SmartDashboard.putString(prefix + "Reject Reason", "None - accepted");
            SmartDashboard.putNumber(prefix + "Submitted X",        poseToSubmit.getX());
            SmartDashboard.putNumber(prefix + "Submitted Y",        poseToSubmit.getY());
            SmartDashboard.putNumber(prefix + "Submitted Rotation", poseToSubmit.getRotation().getDegrees());
        }
    }

    // -------------------------------------------------------------------------
    // UTILITY
    // -------------------------------------------------------------------------

    /** Returns true if we've seen a tag recently (within 0.5s). */
    public boolean hasRecentTarget() {
        return (Timer.getFPGATimestamp() - timeAtLastSeen) < 0.5;
    }

    /**
     * Enable or disable vision heading correction at runtime.
     * Can be toggled from SmartDashboard or via command.
     */
    public void setUpdateHeadingWithVision(boolean update) {
        updateHeadingWithVision = update;
        SmartDashboard.putBoolean("Vision/Update Heading With Vision", updateHeadingWithVision);
    }
}