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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

public class Vision {

    private final SwerveSubsystem drivebase;

    PhotonCamera frontCameraLeft  = new PhotonCamera("frontCameraLeft");
    PhotonCamera frontCameraRight = new PhotonCamera("frontCameraRight");

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

    public double visionMaxTagDist = 15.5;
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

    public void updateVision() {
        if (Robot.isSimulation() && visionSim != null)
            visionSim.update(drivebase.getSwerveDrive().getPose());

        integrateCamera(frontCameraRight, frontRightEstimator, frontCameraRightDebugField, "FrontRight");
        integrateCamera(frontCameraLeft,  frontLeftEstimator,  frontCameraLeftDebugField,  "FrontLeft");

        fusedOdometryField.setRobotPose(drivebase.getSwerveDrive().getPose());
        SmartDashboard.putNumber("Vision/Time Since Last Tag", Timer.getFPGATimestamp() - timeAtLastSeen);
        SmartDashboard.putBoolean("Vision/Recently Saw Tag",   hasRecentTarget());
    }

    private void integrateCamera(PhotonCamera camera, PhotonPoseEstimator estimator,
                                  Field2d debugField, String cameraLabel) {

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {

            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> photonPose = estimator.update(result);
            if (photonPose.isEmpty()) continue;

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
                erp.timestampSeconds, result.getTargets().size()));
            System.out.println(String.format("[%s] ODO Pose: X=%.3f Y=%.3f rot=%.1fdeg",
                cameraLabel, odoPose.getX(), odoPose.getY(), odoPose.getRotation().getDegrees()));

            double tagDist = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Tag Distance", tagDist);

            if (tagDist >= visionMaxTagDist) {
                SmartDashboard.putBoolean("Vision/" + cameraLabel + "/ACCEPTED", false);
                System.out.println("  [" + cameraLabel + "] Rejected - tag too far: " + tagDist);
                continue;
            }

            SmartDashboard.putBoolean("Vision/" + cameraLabel + "/ACCEPTED", true);

            // -----------------------------------------------------------------
            // KEY FIX: Use resetOdometry instead of addVisionMeasurement.
            //
            // addVisionMeasurement was corrupting the heading even with 9999
            // rotation std devs because YAGSL's internal pose estimator was
            // still blending in the rotation component.
            //
            // resetOdometry with vision X/Y + current gyro rotation is 100%
            // guaranteed to never touch the heading. Vision only corrects
            // position, gyro owns rotation completely.
            // -----------------------------------------------------------------
            Pose2d correctedPose = new Pose2d(
                pvPose.getTranslation(), // X/Y from vision
                odoPose.getRotation()    // rotation always from gyro, never vision
            );

            drivebase.getSwerveDrive().resetOdometry(correctedPose);
            timeAtLastSeen = Timer.getFPGATimestamp();

            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Submitted X", correctedPose.getX());
            SmartDashboard.putNumber("Vision/" + cameraLabel + "/Submitted Y", correctedPose.getY());
            SmartDashboard.putString("Vision/" + cameraLabel + "/Status",      "ACCEPTED - pose reset");

            System.out.println(String.format("  [%s] Pose reset X=%.3f Y=%.3f rot=%.1fdeg (gyro heading kept)",
                cameraLabel, correctedPose.getX(), correctedPose.getY(), correctedPose.getRotation().getDegrees()));
        }
    }

    public boolean hasRecentTarget() {
        return (Timer.getFPGATimestamp() - timeAtLastSeen) < 0.5;
    }
}