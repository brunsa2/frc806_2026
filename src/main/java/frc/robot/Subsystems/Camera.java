package frc.robot.Subsystems;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static edu.wpi.first.wpilibj.Timer.getTimestamp;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    private class AprilTag {
        public PoseEstimator<SwerveModulePosition[]> poseEstimator;
        public double timestamp;
    }
    
    private final PhotonCamera camera;
    private final int maxTrackedTargets;
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> positionSupplier;
    private final Map<Integer, AprilTag> aprilTags = new HashMap<>();
    private final SwerveDriveKinematics kinematics;
    private int closestFiducialId;
    private final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(Constants.Camera.FieldLayout, Constants.Camera.RobotToCamera);
    private final SwerveDrivePoseEstimator poseEstimator;

    private final double TIMEOUT = 3;

    private final Alert noTagAlert = new Alert("No April Tags detected", AlertType.kWarning);
    private final Alert tagCountAlert = new Alert("", AlertType.kInfo);



    private final Field2d m_field = new Field2d();



    public Camera(String name, int maxTrackedTargets, Supplier<SwerveDriveKinematics> kinematicsSupplier, Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> positionProvider) {
        camera = new PhotonCamera(name);
        this.maxTrackedTargets = maxTrackedTargets;
        kinematics = kinematicsSupplier.get();
        this.rotationSupplier = rotationSupplier;
        this.positionSupplier = positionProvider;
        this.closestFiducialId = -1;

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rotationSupplier.get(),
            positionSupplier.get(),
            new Pose2d(),
            VecBuilder.fill(Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.AngleStdDev),
            VecBuilder.fill(Constants.Drivetrain.Vision.PositionStdDev, Constants.Drivetrain.Vision.PositionStdDev, 999999)
        );


        SmartDashboard.putData("Field", m_field);
    }
 
    @Override
    public void periodic() {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            var visionEstimate = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEstimate.isEmpty()) {
                visionEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            

            Matrix<N3, N1> stdDevs = visionEstimate.map(e -> getStdDevs(e, result.getTargets())).orElse(Constants.Camera.SingleTagStdDevs);

            SmartDashboard.putNumber("stddev", stdDevs.get(0, 0));

            visionEstimate.ifPresent(e -> {
                poseEstimator.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds, stdDevs);
            });
        }

        poseEstimator.update(rotationSupplier.get(), positionSupplier.get());

        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("T", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        var estStdDevs = Constants.Camera.SingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags > 0) {
            avgDist /= numTags;
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = Constants.Camera.MultiTagStdDevs;
            } 
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
        }
        
        return estStdDevs;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
