package frc.robot.Subsystems;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static edu.wpi.first.wpilibj.Timer.getTimestamp;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    private class AprilTag {
        public PoseEstimator<SwerveModulePosition[]> poseEstimator;
        public double timestamp;
    }
    
    private final PhotonCamera camera;
    private final int maxTrackedTargets;
    private final Supplier<Rotation2d> rotationProvider;
    private final Supplier<SwerveModulePosition[]> positionProvider;
    private final Map<Integer, AprilTag> aprilTags = new HashMap<>();
    private final SwerveDriveKinematics kinematics;
    private int closestFiducialId;

    private final double TIMEOUT = 3;

    private final Alert noTagAlert = new Alert("No April Tags detected", AlertType.kWarning);
    private final Alert tagCountAlert = new Alert("", AlertType.kInfo);

    public Camera(String name, int maxTrackedTargets, Supplier<SwerveDriveKinematics> kinematicsSupplier, Supplier<Rotation2d> rotationProvider, Supplier<SwerveModulePosition[]> positionProvider) {
        camera = new PhotonCamera(name);
        this.maxTrackedTargets = maxTrackedTargets;
        this.kinematics = kinematicsSupplier.get();
        this.rotationProvider = rotationProvider;
        this.positionProvider = positionProvider;
    }
 
    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            double timestamp = result.getTimestampSeconds();
            List<PhotonTrackedTarget> targets = new ArrayList<>(result.getTargets());
            
            // Use filtering like in sample code
                                                                                                                    
            targets.sort((a, b) -> {                                                                                      
                double distA = a.getBestCameraToTarget().getTranslation().getNorm();                                      
                double distB = b.getBestCameraToTarget().getTranslation().getNorm();
                return Double.compare(distA, distB);
            });

            if (targets.size() > maxTrackedTargets) {
                targets.subList(maxTrackedTargets, targets.size()).clear();
            }

            int closestFiducialId = -1;

            for (var target : targets) {
                int fiducialId = target.fiducialId;

                if (target.getPoseAmbiguity() > Constants.Camera.MaxTargetPoseAmbiguity || fiducialId < 0) {
                    continue;
                }

                if (closestFiducialId < 0) {
                    closestFiducialId = target.fiducialId;
                }

                var translation = target.getBestCameraToTarget();
                var cameraPose = new Pose2d(-translation.getX(), -translation.getY(), new Rotation2d(-target.getYaw()));
                AprilTag aprilTag;
                // TODO: check if within one meter of current estimate before adding vision update, is this necessary? what if we lose too far and need a reset?
                if (!aprilTags.containsKey(target.fiducialId)) {
                    aprilTag = new AprilTag();
                    aprilTag.poseEstimator = new SwerveDrivePoseEstimator(
                        kinematics,
                        rotationProvider.get(),
                        positionProvider.get(),
                        cameraPose,
                        new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.AngleStdDev}),
                        new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{Constants.Drivetrain.Vision.XConstantStdDev, Constants.Drivetrain.Vision.YConstantStdDev, Constants.Drivetrain.Vision.AngleStdDev})
                    );
                    aprilTags.put(target.fiducialId, aprilTag);
                } else {
                    aprilTag = aprilTags.get(fiducialId);
                    // Look at sample code, use transform inverse or something against a blank pose3d or post2d?
                    // Tranform camera to robot
                    aprilTag.poseEstimator.addVisionMeasurement(cameraPose, timestamp);
                }
                aprilTag.timestamp = timestamp;
            }

            Iterator<Map.Entry<Integer, AprilTag>> it = aprilTags.entrySet().iterator();
            while (it.hasNext()) {
                Map.Entry<Integer, AprilTag> entry = it.next();
                if (timestamp - entry.getValue().timestamp > TIMEOUT) {
                    it.remove();
                }
                entry.getValue().poseEstimator.update(rotationProvider.get(), positionProvider.get());
            }

            if (aprilTags.size() == 0) {
                noTagAlert.set(true);
                tagCountAlert.set(false);
            } else {
                tagCountAlert.setText("" + aprilTags.size() + " April Tags detected");
                tagCountAlert.set(true);
                noTagAlert.set(false);
            }

            this.closestFiducialId = closestFiducialId;
        }
    }

    public int getClosestFiducialId() {
        return closestFiducialId;
    }

    public Pose2d getClosestPose() {
        return aprilTags.get(closestFiducialId).poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPose(int fiducialId) {
        if (aprilTags.containsKey(fiducialId)) {
            return aprilTags.get(fiducialId).poseEstimator.getEstimatedPosition();
        } else {
            return null;
        }
    }
}
