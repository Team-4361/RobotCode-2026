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
    // SETTINGS
    // -------------------------------------------------------------------------

    public boolean updateHeadingWithVision = false;
    public double  timeAtLastSeen          = 0.0;

    // Maximum tag distance — only filter we keep from the old code
    public double visionMaxTagDist = 15.5;

    // -------------------------------------------------------------------------
    // DEBUG FIELDS
    // -------------------------------------------------------------------------

    Field2d frontCameraRightDebugField = new Field2d();
    Field2d frontCameraLeftDebugField  = new Field2d();
    Field2d fusedOdometryField         = new Field2d();

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

        integrateCamera(frontCameraRight, frontRightEstimator, frontCameraRightDebugField, "FrontRight");
        integrateCamera(frontCameraLeft,  frontLeftEstimator,  frontCameraLeftDebugField,  "FrontLeft");

        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().getPose());

        SmartDashboard.putNumber("Vision/Time Since Last Tag", Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",   hasRecentTarget());
    }

    // -------------------------------------------------------------------------
    // CAMERA PROCESSING — ported directly from old working integrateCamera()
    // -------------------------------------------------------------------------

    private void integrateCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                  Field2d debugField, String cameraLabel) {

        List<PhotonPipelineResult> cameraPipeline = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : cameraPipeline) {

            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> photonPose = estimator.update(result);
            if (photonPose.isEmpty()) continue;

            EstimatedRobotPose erp     = photonPose.get();
            Pose2d pvPose              = erp.estimatedPose.toPose2d();
            Pose2d odoPose             = drivebase.getSwerveDrive().getPose();

            // Update field visualization
            debugField.setRobotPose(pvPose);

            // Dashboard debug values
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw X",        pvPose.getX());
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw Y",        pvPose.getY());
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw Rotation", pvPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Odo X Diff",   pvPose.getX() - odoPose.getX());
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Odo Y Diff",   pvPose.getY() - odoPose.getY());

            // Console debug — same as old code
            System.out.println(String.format("[%s] PV Pose: X=%.3f Y=%.3f rot=%.1fdeg ts=%.3f tags=%d",
                cameraLabel,
                pvPose.getX(), pvPose.getY(), pvPose.getRotation().getDegrees(),
                erp.timestampSeconds, result.getTargets().size()));
            System.out.println(String.format("[%s] ODO Pose: X=%.3f Y=%.3f rot=%.1fdeg",
                cameraLabel,
                odoPose.getX(), odoPose.getY(), odoPose.getRotation().getDegrees()));

            double  tagDist        = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            double  poseAmbiguity  = result.getBestTarget().getPoseAmbiguity();
            int     targetCount    = result.getTargets().size();
            boolean multipleTargets = targetCount > 1;

            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Tag Distance",   tagDist);
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Pose Ambiguity", poseAmbiguity);
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Target Count",   targetCount);

            // Ported directly from old code:
            // alwaysAccept = true means we always fuse regardless of distance or diff.
            // Only keeping tag distance as a sanity check.
            boolean distanceOK   = tagDist < visionMaxTagDist;
            boolean alwaysAccept = true;

            SmartDashboard.putBoolean("Vision/" + cameraLabel + "/Distance OK", distanceOK);
            SmartDashboard.putBoolean("Vision/" + cameraLabel + "/ACCEPTED",    distanceOK || alwaysAccept);

            if (distanceOK || alwaysAccept) {

                timeAtLastSeen = Timer.getFPGATimestamp();

                Pose2d visionPoseToUse;
                if (updateHeadingWithVision) {
                    // Use vision X, Y, AND rotation
                    visionPoseToUse = pvPose;
                } else {
                    // Use vision X, Y but keep current gyro rotation — same as old code
                    visionPoseToUse = new Pose2d(
                        pvPose.getTranslation(),
                        odoPose.getRotation()
                    );
                }

                // Std devs ported directly from old code — 9999 for rotation
                // when not using vision heading, same values as old working version
                Matrix<N3, N1> stdDevs;
                if (updateHeadingWithVision) {
                    stdDevs = multipleTargets
                        ? VecBuilder.fill(0.5, 0.5, 0.7)
                        : VecBuilder.fill(0.9, 0.9, 1.2);
                } else {
                    stdDevs = multipleTargets
                        ? VecBuilder.fill(0.5, 0.5, 9999)
                        : VecBuilder.fill(0.9, 0.9, 9999);
                }

                drivebase.getSwerveDrive().addVisionMeasurement(
                    visionPoseToUse,
                    erp.timestampSeconds,
                    stdDevs
                );

                System.out.println("  [" + cameraLabel + "] Vision measurement added ("
                    + (updateHeadingWithVision ? "with vision rotation" : "with gyro rotation") + ")");

            } else {
                System.out.println("  [" + cameraLabel + "] Vision rejected - dist:" + distanceOK);
            }
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