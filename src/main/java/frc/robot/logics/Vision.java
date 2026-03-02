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

    // Both cameras: front-facing, center-mounted, 10° upward pitch.
    //
    // Positive X = forward, positive Y = left, positive Z = up.
    // Positive pitch = angled upward (toward ceiling).
    //
    // Adjust the Translation3d values to match your actual mounting position.
    // Currently assumes both cameras are at the front of the robot, side by side,
    // at the same height. Tweak Y offset if they are offset left/right.
    Transform3d frontCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.0),   // Forward from robot center
            Units.inchesToMeters(3.0),    // Slightly left of center (adjust as needed)
            Units.inchesToMeters(22.5)    // Height above ground
        ),
        new Rotation3d(
            0,                                        // Roll  (0 = level)
            Units.degreesToRadians(-25),               // Pitch (positive = angled up)
            Units.degreesToRadians(0)                 // Yaw   (0 = facing forward)
        )
    );

    Transform3d backCameraTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.0),   // Also forward-facing (second front cam)
            Units.inchesToMeters(-3.0),   // Slightly right of center (adjust as needed)
            Units.inchesToMeters(14.5)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-25),   // Same 10° upward pitch
            Units.degreesToRadians(0)     // Also facing forward
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

    // -------------------------------------------------------------------------
    // SIMULATION
    // -------------------------------------------------------------------------

    // These are only initialized when running in simulation (Robot.isSimulation()).
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

        // OV9281 camera properties:
        //   - 1280x800 resolution
        //   - 70° horizontal FOV
        //   - 100fps capability; we simulate at a realistic 45fps
        //   - ~35ms average latency with some jitter
        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(70)); // width, height, horizontal FOV
        cameraProps.setCalibError(0.20, 0.06);  // avg pixel error, std dev — OV9281 is a sharp sensor
        cameraProps.setFPS(45);                  // conservative sim fps (camera can do 100 but coprocessor limits it)
        cameraProps.setAvgLatencyMs(35);         // realistic USB + PhotonVision processing latency
        cameraProps.setLatencyStdDevMs(8);       // some jitter

        // Create simulated cameras using the same PhotonCamera objects the real code uses.
        // This means processCamera() works IDENTICALLY in sim — no code path changes.
        frontCameraSim = new PhotonCameraSim(frontCamera, cameraProps);
        backCameraSim  = new PhotonCameraSim(backCamera,  cameraProps);

        visionSim.addCamera(frontCameraSim, frontCameraTransform);
        //visionSim.addCamera(backCameraSim,  backCameraTransform);

        // Draw detected targets as wireframes in Glass / Shuffleboard simulation view
        frontCameraSim.enableDrawWireframe(true);
        backCameraSim.enableDrawWireframe(true);
    }

    // -------------------------------------------------------------------------
    // MAIN UPDATE - Call this from Robot.periodic() or a subsystem periodic
    // -------------------------------------------------------------------------

    public void updateVision() {
        // In simulation: feed the current robot pose into the virtual cameras FIRST,
        // so that getAllUnreadResults() returns realistic simulated detections.
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(drivebase.getSwerveDrive().getPose());
        }

        processCamera(frontCamera, frontEstimator, frontCameraDebugField, "Front");
        processCamera(backCamera,  backEstimator,  backCameraDebugField,  "Back");

        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().getPose());

        SmartDashboard.putNumber("Vision/Time Since Last Tag", Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",   hasRecentTarget());
    }

    // -------------------------------------------------------------------------
    // CAMERA PROCESSING
    // -------------------------------------------------------------------------

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                Field2d debugField, String cameraLabel) {

        String prefix = "Vision/" + cameraLabel + "/";

        // FIX: Set reference pose each loop so single-tag fallback picks the
        // solution closest to where we already think we are (avoids mirror-image flips).
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

            // FIX: getBestTarget() can return null in rare edge cases even when
            // hasTargets() is true. Pull it out and guard against null explicitly.
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
            boolean diffOK      = poseDifference < MAX_VISION_ODO_DIFF_METERS;
            boolean ambiguityOK = isMultiTag || (poseAmbiguity < MAX_POSE_AMBIGUITY && poseAmbiguity >= 0);

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

    /**
     * Enable or disable vision heading correction at runtime.
     * Can be toggled from SmartDashboard or via command.
     */
    public void setUpdateHeadingWithVision(boolean update) {
        updateHeadingWithVision = update;
        SmartDashboard.putBoolean("Vision/Update Heading With Vision", updateHeadingWithVision);
    }
}