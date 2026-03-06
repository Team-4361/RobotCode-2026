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
    // TUNING CONSTANTS
    // -------------------------------------------------------------------------

    // Maximum distance (meters) from a tag before we reject the measurement.
    private static final double MAX_TAG_DISTANCE_METERS = 15.5;

    // Pose ambiguity threshold — only applied for single tag readings.
    private static final double MAX_POSE_AMBIGUITY = 0.15;

    // Set to true ONLY if you want vision to also correct the gyro heading.
    public boolean updateHeadingWithVision = false;

    // -------------------------------------------------------------------------
    // CAMERA DEFINITIONS
    // -------------------------------------------------------------------------

    PhotonCamera frontCameraLeft  = new PhotonCamera("frontCameraLeft");
    PhotonCamera frontCameraRight = new PhotonCamera("frontCameraRight");

    Transform3d frontCameraLeftTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5),
            Units.inchesToMeters(-8.75),
            Units.inchesToMeters(19)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-26),
            Units.degreesToRadians(0)
        )
    );

    Transform3d frontCameraRightTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5),
            Units.inchesToMeters(8.75),
            Units.inchesToMeters(22.125)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-26),
            Units.degreesToRadians(0)
        )
    );

    // -------------------------------------------------------------------------
    // POSE ESTIMATORS
    // -------------------------------------------------------------------------

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonPoseEstimator frontRightEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCameraRightTransform
    );

    PhotonPoseEstimator frontLeftEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontCameraLeftTransform
    );

    // -------------------------------------------------------------------------
    // DEBUG FIELDS
    // -------------------------------------------------------------------------

    Field2d frontCameraRightDebugField = new Field2d();
    Field2d frontCameraLeftDebugField  = new Field2d();
    Field2d fusedOdometryField         = new Field2d();

    public double timeAtLastSeen = 0.0;

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

        SmartDashboard.putData("Vision/FrontCamLeft Raw Pose",  frontCameraLeftDebugField);
        SmartDashboard.putData("Vision/FrontCamRight Raw Pose", frontCameraRightDebugField);
        SmartDashboard.putData("Vision/Fused Odometry Pose",    fusedOdometryField);
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

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        cameraProps.setCalibError(0.20, 0.06);
        cameraProps.setFPS(45);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(8);

        frontCameraSim = new PhotonCameraSim(frontCameraLeft,  cameraProps);
        backCameraSim  = new PhotonCameraSim(frontCameraRight, cameraProps);

        // FIX: Each sim camera now uses its own correct transform
        visionSim.addCamera(frontCameraSim, frontCameraLeftTransform);
        visionSim.addCamera(backCameraSim,  frontCameraRightTransform);

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

        processCamera(frontCameraRight, frontRightEstimator, frontCameraRightDebugField, "FrontRight");
        processCamera(frontCameraLeft,  frontLeftEstimator,  frontCameraLeftDebugField,  "FrontLeft");

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

            double  tagDist       = bestTarget.getBestCameraToTarget().getTranslation().getNorm();
            double  poseAmbiguity = bestTarget.getPoseAmbiguity();
            int     targetCount   = result.getTargets().size();
            boolean isMultiTag    = targetCount > 1;

            SmartDashboard.putNumber(prefix + "Tag Distance (m)", tagDist);
            SmartDashboard.putNumber(prefix + "Pose Ambiguity",   poseAmbiguity);
            SmartDashboard.putNumber(prefix + "Target Count",     targetCount);
            SmartDashboard.putBoolean(prefix + "Is Multi-Tag",    isMultiTag);

            // Only two filters — tag distance and single-tag ambiguity.
            // No odometry diff check at all, so vision always fuses in
            // regardless of how far the pose is from current odometry.
            boolean distOK      = tagDist < MAX_TAG_DISTANCE_METERS;
            boolean ambiguityOK = isMultiTag || (poseAmbiguity >= 0 && poseAmbiguity < MAX_POSE_AMBIGUITY);
            boolean accepted    = distOK && ambiguityOK;

            SmartDashboard.putBoolean(prefix + "Distance OK",  distOK);
            SmartDashboard.putBoolean(prefix + "Ambiguity OK", ambiguityOK);
            SmartDashboard.putBoolean(prefix + "ACCEPTED",     accepted);

            if (!accepted) {
                SmartDashboard.putString(prefix + "Reject Reason",
                    !distOK ? "Tag too far" : "High ambiguity");
                continue;
            }

            // Always use gyro heading unless updateHeadingWithVision is on.
            // Vision only corrects X and Y position.
            Pose2d poseToSubmit;
            if (updateHeadingWithVision) {
                poseToSubmit = visionPose;
            } else {
                poseToSubmit = new Pose2d(
                    visionPose.getTranslation(),
                    odoPose.getRotation()  // keep gyro heading, only correct X/Y
                );
            }

            // Scale X/Y trust by distance — farther tag = less trust.
            // Rotation std dev is Double.MAX_VALUE when not using vision heading
            // so the Kalman filter cannot touch the heading no matter what.
            double distanceScaleFactor = 1.0 + (tagDist / MAX_TAG_DISTANCE_METERS);

            Matrix<N3, N1> finalStdDevs;
            if (updateHeadingWithVision) {
                finalStdDevs = VecBuilder.fill(
                    (isMultiTag ? 0.4 : 0.8) * distanceScaleFactor,
                    (isMultiTag ? 0.4 : 0.8) * distanceScaleFactor,
                     isMultiTag ? 0.6 : 1.2
                );
            } else {
                finalStdDevs = VecBuilder.fill(
                    (isMultiTag ? 0.5 : 0.9) * distanceScaleFactor,
                    (isMultiTag ? 0.5 : 0.9) * distanceScaleFactor,
                    Double.MAX_VALUE  // never touch heading
                );
            }

            drivebase.getSwerveDrive().addVisionMeasurement(
                poseToSubmit,
                estimate.timestampSeconds,
                finalStdDevs
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

    /** Enable or disable vision heading correction at runtime. */
    public void setUpdateHeadingWithVision(boolean update) {
        updateHeadingWithVision = update;
        SmartDashboard.putBoolean("Vision/Update Heading With Vision", updateHeadingWithVision);
    }
}