package frc.robot.logics;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;
import java.util.Optional;
import org.photonvision.simulation.SimCameraProperties;
import frc.robot.Robot;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {

    // --- Drivebase ---
    private final SwerveSubsystem drivebase;

    // --- Feature flags ---
    boolean updateHeadingWithVision = true;
    boolean useFrontLeft  = true;
    boolean useFrontRight = true;

    // --- AprilTag field layout ---
    AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // --- Cameras ---
    PhotonCamera frontLeftCam  = new PhotonCamera("frontLeftCam");
    PhotonCamera frontRightCam = new PhotonCamera("frontRightCam");

    // Timestamp of the last AprilTag observation
    public double timeATLastSeen = 0.0;

    // Max distance to accept a tag pose measurement (metres)
    public double visionMaxATDist = 10.0;

    // --- Camera-to-robot transforms ---
    Transform3d frontLeftCamTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(13.5),
            Units.inchesToMeters(-8.75),
            Units.inchesToMeters(19.0)
        ),
        new Rotation3d(0, Units.degreesToRadians(-26.8), Units.degreesToRadians(0))
    );

    Transform3d frontRightCamTransform = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(15),
            Units.inchesToMeters(13.9375),
            Units.inchesToMeters(19.0625)
        ),
        new Rotation3d(0, Units.degreesToRadians(-26.8), Units.degreesToRadians(0))
    );

    // --- Pose estimators ---
    PhotonPoseEstimator frontLeftPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontLeftCamTransform
    );

    PhotonPoseEstimator frontRightPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        frontRightCamTransform
    );

    // --- SmartDashboard Field2d views ---
    Field2d photonField_frontLeft  = new Field2d();
    Field2d photonField_frontRight = new Field2d();

    // --- Simulation ---
    VisionSystemSim visionSim;
    PhotonCameraSim frontLeftCamSim;
    PhotonCameraSim frontRightCamSim;

    public Vision(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        frontLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        frontRightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putData("PhotonPose FrontLeft",  photonField_frontLeft);
        SmartDashboard.putData("PhotonPose FrontRight", photonField_frontRight);

        if (Robot.isSimulation()) {
            setupSimulation();
        }
    }

    private void setupSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagFieldLayout);

        // OV9281 properties at 1280x800
        // The OV9281 is a global-shutter monochrome sensor, so:
        //   - Very low latency (~5ms exposure)
        //   - Minimal noise
        //   - No rolling shutter distortion
        SimCameraProperties ov9281Props = new SimCameraProperties();
        ov9281Props.setCalibration(1280, 800, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(79.0));
        // OV9281 has very low noise due to global shutter — keep error tight
        ov9281Props.setCalibError(0.8, 0.1); // 0.8px reprojection error, 0.1px stdev
        // ~55 fps average across your 50-60fps range
        ov9281Props.setFPS(55);
        // Global shutter = very short exposure, minimal motion blur
        ov9281Props.setAvgLatencyMs(30);   // typical PhotonVision pipeline latency on Orange Pi
        ov9281Props.setLatencyStdDevMs(3); // low variance due to consistent pipeline timing

        frontLeftCamSim  = new PhotonCameraSim(frontLeftCam,  ov9281Props);
        frontRightCamSim = new PhotonCameraSim(frontRightCam, ov9281Props);

        // Enable the wireframe target view in the sim GUI so you can
        // visually verify tag detection in the camera streams
        frontLeftCamSim.enableDrawWireframe(true);
        frontRightCamSim.enableDrawWireframe(true);

        visionSim.addCamera(frontLeftCamSim,  frontLeftCamTransform);
        visionSim.addCamera(frontRightCamSim, frontRightCamTransform);
    }

    public void updatePhotonVision() {
        if (Robot.isSimulation()) {
            // Feed the sim drivetrain pose into the vision sim so cameras
            // see tags from the correct position each loop
            drivebase.getSwerveDrive().getSimulationDriveTrainPose().ifPresent(
                simPose -> visionSim.update(simPose)
            );
        }

        integrateCamera(useFrontLeft,  frontLeftCam,  frontLeftPoseEstimator,
                        photonField_frontLeft,  visionMaxATDist);
        integrateCamera(useFrontRight, frontRightCam, frontRightPoseEstimator,
                        photonField_frontRight, visionMaxATDist);
    }

    public void integrateCamera(
            boolean useCamera,
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            Field2d photonField,
            double maxDistance) {

        for (var result : camera.getAllUnreadResults()) {
            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> photonPose = estimator.update(result);

            if (photonPose.isPresent()) {
                photonField.setRobotPose(photonPose.get().estimatedPose.toPose2d());

                double bestTagDist = result.getBestTarget()
                    .bestCameraToTarget
                    .getTranslation()
                    .getNorm();

                double poseAmbiguity = result.getBestTarget().getPoseAmbiguity();

                SmartDashboard.putNumber(camera.getName() + " BestTagDist", bestTagDist);
                SmartDashboard.putNumber(camera.getName() + " Ambiguity",   poseAmbiguity);

                if (useCamera && bestTagDist < maxDistance && poseAmbiguity < 0.15) {
                    if (updateHeadingWithVision) {
                        drivebase.getSwerveDrive().addVisionMeasurement(
                            photonPose.get().estimatedPose.toPose2d(),
                            photonPose.get().timestampSeconds
                        );
                        timeATLastSeen = Timer.getFPGATimestamp();
                    }
                }
            }
        }
    }
}