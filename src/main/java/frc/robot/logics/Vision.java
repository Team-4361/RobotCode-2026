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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

public class Vision {

    private final SwerveSubsystem drivebase;

    PhotonCamera frontCameraLeft  = new PhotonCamera("frontCameraLeft");
    PhotonCamera frontCameraRight = new PhotonCamera("frontCameraRight");
    // ========== FIELD CONSTANTS ==========
    private static final double FIELD_LENGTH_M = Units.inchesToMeters(651.25);
    private static final double FIELD_WIDTH_M = Units.inchesToMeters(317.69);

    public static final Translation2d HUB_CENTER_BLUE =
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

    public static final Translation2d HUB_CENTER_RED =
        new Translation2d(FIELD_LENGTH_M - Units.inchesToMeters(182.11),
                          Units.inchesToMeters(158.84));

    Transform3d frontCameraLeftTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5),
            Units.inchesToMeters(-8.75),
            Units.inchesToMeters(19)
        ),
        new Rotation3d(0, Units.degreesToRadians(-26), Units.degreesToRadians(0))
    );

    Transform3d frontCameraRightTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5),
            Units.inchesToMeters(8.75),
            Units.inchesToMeters(22.125)
        ),
        new Rotation3d(0, Units.degreesToRadians(-26), Units.degreesToRadians(0))
    );

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonPoseEstimator frontRightEstimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCameraRightTransform);

    PhotonPoseEstimator frontLeftEstimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCameraLeftTransform);

    public double visionMaxTagDist = 5.5;
    public double timeAtLastSeen   = 0.0;

    Field2d frontCameraRightDebugField = new Field2d();
    Field2d frontCameraLeftDebugField  = new Field2d();
    Field2d fusedOdometryField         = new Field2d();

    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim backCameraSim;

    public Vision(SwerveSubsystem drivebaseIn) {
        this.drivebase = drivebaseIn;
        SmartDashboard.putData("Vision/FrontCamLeft Raw Pose",  frontCameraLeftDebugField);
        SmartDashboard.putData("Vision/FrontCamRight Raw Pose", frontCameraRightDebugField);
        SmartDashboard.putData("Vision/Fused Odometry Pose",    fusedOdometryField);
        if (Robot.isSimulation()) setupSimulation();
    }

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

    /**
     * Returns the hub center for the current alliance.
     * Falls back to blue if the alliance is not yet determined.
     */
    public Translation2d getHubCenter() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return HUB_CENTER_RED;
        }
        return HUB_CENTER_BLUE;
    }

    public void updateVision() {
        if (Robot.isSimulation() && visionSim != null)
            visionSim.update(drivebase.getSwerveDrive().getPose());

        integrateCamera(frontCameraRight, frontRightEstimator, frontCameraRightDebugField, "FrontRight");
        integrateCamera(frontCameraLeft,  frontLeftEstimator,  frontCameraLeftDebugField,  "FrontLeft");

        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().swerveDrivePoseEstimator.getEstimatedPosition());

        // Publish alliance and hub center for dashboard visibility
        Optional<Alliance> alliance = DriverStation.getAlliance();
        SmartDashboard.putString("Vision/Alliance", alliance.map(Enum::name).orElse("UNKNOWN"));
        Translation2d hub = getHubCenter();
        SmartDashboard.putNumber("Vision/Hub Center X (m)", hub.getX());
        SmartDashboard.putNumber("Vision/Hub Center Y (m)", hub.getY());

        SmartDashboard.putNumber("Vision/Time Since Last Tag", Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",   hasRecentTarget());
    }

    private void integrateCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                  Field2d debugField, String cameraLabel) {

        List<PhotonPipelineResult> cameraPipeline = camera.getAllUnreadResults();

        // Update the estimator with every result so its internal state stays
        // current, but only submit a vision measurement for the last result.
        // Submitting multiple measurements per loop tick hammers the Kalman
        // filter and causes visible drive stutter.
        Optional<EstimatedRobotPose> photonPose = Optional.empty();
        for (int i = 0; i < cameraPipeline.size(); i++) {
            PhotonPipelineResult result = cameraPipeline.get(i);

            // Skip if no targets
            if (!result.hasTargets()) {
                continue;
            }

            photonPose = estimator.update(result);
        }

        // Nothing usable in this batch — bail out
        if (photonPose.isEmpty()) return;

        // Use the last result for tag distance check and dashboard/submission
        PhotonPipelineResult lastResult = cameraPipeline.get(cameraPipeline.size() - 1);
        if (!lastResult.hasTargets()) return;

        EstimatedRobotPose erp = photonPose.get();
        Pose2d pvPose  = erp.estimatedPose.toPose2d();
        Pose2d odoPose = drivebase.getSwerveDrive().getPose();

        debugField.setRobotPose(pvPose);

        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw X",        pvPose.getX());
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw Y",        pvPose.getY());
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Raw Rotation", pvPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Odo X Diff",   pvPose.getX() - odoPose.getX());
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Odo Y Diff",   pvPose.getY() - odoPose.getY());

        System.out.println(String.format("[%s] PV Pose: X=%.3f Y=%.3f rot=%.1fdeg ts=%.3f tags=%d",
            cameraLabel, pvPose.getX(), pvPose.getY(), pvPose.getRotation().getDegrees(),
            erp.timestampSeconds, lastResult.getTargets().size()));
        System.out.println(String.format("[%s] ODO Pose: X=%.3f Y=%.3f rot=%.1fdeg",
            cameraLabel, odoPose.getX(), odoPose.getY(), odoPose.getRotation().getDegrees()));

        double tagDist = lastResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Tag Distance", tagDist);

        if (tagDist >= visionMaxTagDist) {
            SmartDashboard.putBoolean("Vision/" + cameraLabel + "/ACCEPTED", false);
            System.out.println("  [" + cameraLabel + "] Rejected - tag too far: " + tagDist);
            return;
        }

        SmartDashboard.putBoolean("Vision/" + cameraLabel + "/ACCEPTED", true);

        // -----------------------------------------------------------------
        // Hub distance from vision pose, using alliance-correct hub center.
        // -----------------------------------------------------------------
        Translation2d hubCenter = getHubCenter();
        double hubDistM  = pvPose.getTranslation().getDistance(hubCenter);
        double hubDistIn = Units.metersToInches(hubDistM);
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Hub Distance (m)",  hubDistM);
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Hub Distance (in)", hubDistIn);

        // -----------------------------------------------------------------
        // Submit vision measurement using raw gyro heading for rotation.
        // odoPose.getRotation() drifts with wheel odometry over time — if we
        // lock vision to a drifted heading the X/Y estimate drifts too because
        // the tag angle is wrong. The gyro is absolute and drift-free, so it
        // gives vision a stable rotation anchor regardless of odometry error.
        // Rotation std dev is 9999999 as an extra safety net on top of that.
        // -----------------------------------------------------------------
        Pose2d poseToSubmit = new Pose2d(
            pvPose.getTranslation(),
            drivebase.getSwerveDrive().getOdometryHeading()
        );

        drivebase.getSwerveDrive().addVisionMeasurement(
            poseToSubmit,
            erp.timestampSeconds,
            // Lower X/Y std devs = trust vision more = vision actively corrects
            // odometry drift instead of being diluted by it.
            // Scale by tagDist so we trust close tags more than far ones.
            edu.wpi.first.math.VecBuilder.fill(0.05 * tagDist, 0.05 * tagDist, 9999999)
        );

        timeAtLastSeen = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Submitted X", poseToSubmit.getX());
        SmartDashboard.putNumber("Vision/" + cameraLabel + "/Submitted Y", poseToSubmit.getY());
        SmartDashboard.putString("Vision/" + cameraLabel + "/Status",      "ACCEPTED");

        System.out.println(String.format("  [%s] Vision fused X=%.3f Y=%.3f rot=%.1fdeg (gyro heading kept)",
            cameraLabel, poseToSubmit.getX(), poseToSubmit.getY(), poseToSubmit.getRotation().getDegrees()));
    }

    public boolean hasRecentTarget() {
        return (Timer.getFPGATimestamp() - timeAtLastSeen) < 0.5;
    }
}