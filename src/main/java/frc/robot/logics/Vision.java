package frc.robot.logics;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private static final double MAX_TAG_DISTANCE_METERS = 15.5;

    // If the vision pose is more than this far (meters) from odometry, reject it.
    // This check is SKIPPED until odometry has been seeded from vision once.
    private static final double MAX_VISION_ODO_DIFF_METERS = 1.0;

    // Pose ambiguity threshold (0 = perfect, 0.2+ = bad). Only matters for single tags.
    private static final double MAX_POSE_AMBIGUITY = 0.15;

    // Standard deviations for vision measurements [x_meters, y_meters, rotation_radians].
    // High rotation std dev = we barely trust vision rotation (rely on gyro instead).
    private static final Matrix<N3, N1> STD_MULTI_TAG  = VecBuilder.fill(0.5, 0.5, 9999.0);
    private static final Matrix<N3, N1> STD_SINGLE_TAG = VecBuilder.fill(0.9, 0.9, 9999.0);

    // Set to true ONLY if you want vision to also correct the gyro heading.
    public boolean updateHeadingWithVision = false;

    // -------------------------------------------------------------------------
    // CAMERA DEFINITIONS
    // -------------------------------------------------------------------------

    // Camera names must match exactly what's configured in PhotonVision
    PhotonCamera frontCamera = new PhotonCamera("frontCam");
    PhotonCamera backCamera  = new PhotonCamera("backCam");

    Transform3d frontCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.0),
            Units.inchesToMeters(3.0),
            Units.inchesToMeters(22.5)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-25),
            Units.degreesToRadians(0)
        )
    );

    Transform3d backCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.0),
            Units.inchesToMeters(-3.0),
            Units.inchesToMeters(14.5)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-25),
            Units.degreesToRadians(0)
        )
    );

    // -------------------------------------------------------------------------
    // POSE ESTIMATORS
    // -------------------------------------------------------------------------

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

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
    // DEBUG FIELDS
    // -------------------------------------------------------------------------

    Field2d frontCameraDebugField = new Field2d();
    Field2d backCameraDebugField  = new Field2d();
    Field2d fusedOdometryField    = new Field2d();

    public double timeAtLastSeen = 0.0;

    /**
     * Whether we've seeded odometry from vision yet this power cycle / match.
     *
     * Problem this solves:
     *   At match start, odometry is reset to (0,0) or the DS-provided starting pose.
     *   The first vision reading might say the robot is at (8.2, 4.1) — a 9-meter
     *   difference — which would be rejected by MAX_VISION_ODO_DIFF_METERS.
     *   So vision could NEVER correct the initial error.
     *
     * Fix:
     *   Until we get the first valid vision fix (good distance + ambiguity),
     *   skip the odometry-diff check entirely and reset odometry outright to
     *   that first trusted vision pose.  After that, normal filtering resumes.
     */
    private boolean hasSeededOdometry = false;

    // -------------------------------------------------------------------------
    // SIMULATION
    // -------------------------------------------------------------------------

    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim backCameraSim;

    // -------------------------------------------------------------------------
    // CONSTRUCTOR
    // -------------------------------------------------------------------------

    public Vision(SwerveSubsystem drivebaseIn) {
        this.drivebase = drivebaseIn;

        SmartDashboard.putData("Vision/FrontCam Raw Pose",   frontCameraDebugField);
        SmartDashboard.putData("Vision/BackCam Raw Pose",    backCameraDebugField);
        SmartDashboard.putData("Vision/Fused Odometry Pose", fusedOdometryField);
        SmartDashboard.putBoolean("Vision/Update Heading With Vision", updateHeadingWithVision);
        SmartDashboard.putBoolean("Vision/Odometry Seeded", hasSeededOdometry);

        if (Robot.isSimulation()) {
            setupSimulation();
        }
    }

    // -------------------------------------------------------------------------
    // SIMULATION SETUP
    // -------------------------------------------------------------------------

    private void setupSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        cameraProps.setCalibError(0.20, 0.06);
        cameraProps.setFPS(45);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(8);

        frontCameraSim = new PhotonCameraSim(frontCamera, cameraProps);
        backCameraSim  = new PhotonCameraSim(backCamera,  cameraProps);

        visionSim.addCamera(frontCameraSim, frontCameraTransform);
        //visionSim.addCamera(backCameraSim, backCameraTransform);

        frontCameraSim.enableDrawWireframe(true);
        backCameraSim.enableDrawWireframe(true);
    }

    // -------------------------------------------------------------------------
    // MAIN UPDATE
    // -------------------------------------------------------------------------

    public void updateVision() {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(drivebase.getSwerveDrive().getPose());
        }

        processCamera(frontCamera, frontEstimator, frontCameraDebugField, "Front");
        processCamera(backCamera,  backEstimator,  backCameraDebugField,  "Back");

        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().getPose());

        SmartDashboard.putNumber("Vision/Time Since Last Tag", Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",   hasRecentTarget());
        SmartDashboard.putBoolean("Vision/Odometry Seeded",    hasSeededOdometry);
    }

    // -------------------------------------------------------------------------
    // CAMERA PROCESSING
    // -------------------------------------------------------------------------

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                Field2d debugField, String cameraLabel) {

        String prefix = "Vision/" + cameraLabel + "/";

        estimator.setReferencePose(drivebase.getSwerveDrive().getPose());

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {

            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> maybeEstimate = estimator.update(result);
            if (maybeEstimate.isEmpty()) continue;

            EstimatedRobotPose estimate = maybeEstimate.get();
            Pose2d visionPose = estimate.estimatedPose.toPose2d();
            Pose2d odoPose    = drivebase.getSwerveDrive().getPose();

            debugField.setRobotPose(visionPose);
            SmartDashboard.putNumber(prefix + "Raw X",        visionPose.getX());
            SmartDashboard.putNumber(prefix + "Raw Y",        visionPose.getY());
            SmartDashboard.putNumber(prefix + "Raw Rotation", visionPose.getRotation().getDegrees());

            var bestTarget = result.getBestTarget();
            if (bestTarget == null) continue;

            double tagDist        = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
            double poseAmbiguity  = bestTarget.getPoseAmbiguity();
            int    targetCount    = result.getTargets().size();
            double poseDifference = visionPose.getTranslation().getDistance(odoPose.getTranslation());
            boolean isMultiTag    = targetCount > 1;

            SmartDashboard.putNumber(prefix + "Tag Distance (m)", tagDist);
            SmartDashboard.putNumber(prefix + "Pose Ambiguity",   poseAmbiguity);
            SmartDashboard.putNumber(prefix + "Target Count",     targetCount);
            SmartDashboard.putNumber(prefix + "Vs Odo Diff (m)",  poseDifference);
            SmartDashboard.putBoolean(prefix + "Is Multi-Tag",    isMultiTag);

            boolean distOK      = tagDist < MAX_TAG_DISTANCE_METERS;
            boolean ambiguityOK = isMultiTag || (poseAmbiguity < MAX_POSE_AMBIGUITY && poseAmbiguity >= 0);

            // ---------------------------------------------------------------
            // SEEDING LOGIC
            //
            // Before we've had our first valid fix, skip the odometry-diff
            // check entirely (poseDifference could be 10+ meters at startup).
            // Once distOK + ambiguityOK pass, hard-reset odometry to that
            // vision pose so all future diff checks are meaningful.
            // ---------------------------------------------------------------
            boolean diffOK;
            if (!hasSeededOdometry) {
                // Pre-seed: only care about tag distance and ambiguity quality.
                // Ignore poseDifference — odometry isn't trustworthy yet.
                diffOK = true;
                SmartDashboard.putString(prefix + "Diff OK Note", "Pre-seed: diff check skipped");
            } else {
                diffOK = poseDifference < MAX_VISION_ODO_DIFF_METERS;
            }

            boolean accepted = distOK && diffOK && ambiguityOK;

            SmartDashboard.putBoolean(prefix + "Distance OK",  distOK);
            SmartDashboard.putBoolean(prefix + "Diff OK",      diffOK);
            SmartDashboard.putBoolean(prefix + "Ambiguity OK", ambiguityOK);
            SmartDashboard.putBoolean(prefix + "ACCEPTED",     accepted);

            if (!accepted) {
                SmartDashboard.putString(prefix + "Reject Reason",
                    !distOK ? "Tag too far" : !diffOK ? "Jump too large" : "High ambiguity");
                continue;
            }

            // ---------------------------------------------------------------
            // First ever valid reading: hard-reset odometry so the robot
            // knows where it actually is on the field from the start.
            // ---------------------------------------------------------------
            if (!hasSeededOdometry) {
                Pose2d seedPose = new Pose2d(
                    visionPose.getTranslation(),
                    // Keep the current gyro heading — vision X/Y is reliable,
                    // gyro heading at startup is more reliable than vision yaw.
                    odoPose.getRotation()
                );
                drivebase.getSwerveDrive().resetOdometry(seedPose);
                hasSeededOdometry = true;
                SmartDashboard.putBoolean("Vision/Odometry Seeded", true);
                SmartDashboard.putString(prefix + "Reject Reason", "None - SEEDED odometry");
                // Don't also addVisionMeasurement this loop; the reset already placed us correctly.
                // The very next loop iteration will have a tiny poseDifference and proceed normally.
                timeAtLastSeen = Timer.getFPGATimestamp();
                continue;
            }

            // ---------------------------------------------------------------
            // Normal operation (post-seed): fuse into the Kalman filter
            // ---------------------------------------------------------------
            Pose2d poseToSubmit;
            if (updateHeadingWithVision) {
                poseToSubmit = visionPose;
            } else {
                poseToSubmit = new Pose2d(
                    visionPose.getTranslation(),
                    odoPose.getRotation()
                );
            }

            Matrix<N3, N1> stdDevs;
            if (updateHeadingWithVision) {
                stdDevs = isMultiTag
                    ? VecBuilder.fill(0.4, 0.4, 0.6)
                    : VecBuilder.fill(0.8, 0.8, 1.2);
            } else {
                stdDevs = isMultiTag ? STD_MULTI_TAG : STD_SINGLE_TAG;
            }

            // Scale trust by distance - farther tag = less trust
            double distanceScaleFactor = 1.0 + (tagDist / MAX_TAG_DISTANCE_METERS);
            Matrix<N3, N1> scaledStdDevs = VecBuilder.fill(
                stdDevs.get(0, 0) * distanceScaleFactor,
                stdDevs.get(1, 0) * distanceScaleFactor,
                stdDevs.get(2, 0)
            );

            drivebase.getSwerveDrive().addVisionMeasurement(
                poseToSubmit,
                estimate.timestampSeconds,
                scaledStdDevs
            );

            timeAtLastSeen = Timer.getFPGATimestamp();

            SmartDashboard.putString(prefix + "Reject Reason",      "None - accepted");
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

    /** Returns true if vision has successfully seeded odometry this session. */
    public boolean isOdometrySeeded() {
        return hasSeededOdometry;
    }

    /**
     * Call this if odometry is manually reset (e.g. at the start of a new match
     * or during autonomous setup) so vision will re-seed from the next good reading.
     */
    public void resetSeedFlag() {
        hasSeededOdometry = false;
        SmartDashboard.putBoolean("Vision/Odometry Seeded", false);
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