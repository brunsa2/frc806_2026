package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.SwerveModule;

public class Constants {

    public static final double Delta = 1e-2;
    
    // 25.25 inches wide, 18.16 inches deep
    public static final Translation2d[] moduleLocations = {
        new Translation2d(0.230505, -0.320675),  //front right +-
        new Translation2d(0.230505, 0.320675), //front left ++
        new Translation2d(-0.230505, 0.320675),  //rear left -+
        new Translation2d(-0.230505, -0.320675)  //rear right --
    };

    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = Math.sqrt(Math.pow(moduleLocations[1].getX(), 2) + Math.pow(moduleLocations[1].getY(), 2));
    public static final double attainableMaxModuleSpeedMPS = 4.572; // 15 feet/s
    public static final double attainableMaxTranslationalSpeedMPS = attainableMaxModuleSpeedMPS;
    public static final double attainableMaxRotationalVelocityRPS = attainableMaxModuleSpeedMPS/DriveBaseRadius;

    public static final int PigeonID = 22;   

    public static double controllerDeadband = 0.15; 

    public interface Camera {
        public static final String Name = "front";
        public static final int MaxTrackedTargets = 1;
        public static final double MaxTargetPoseAmbiguity = 0.2;
        public static final Translation2d Position = new Translation2d(0, 0);

        // TODO: fix these fake values
        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 999);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final Transform3d RobotToCamera = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        );

        List<AprilTag> Tags = List.of(
            new AprilTag(7, new Pose3d(
                new Translation3d(5, 2.5, 0.3048),
                new Rotation3d(0, 0, Rotation2d.fromDegrees(180).getRadians())
            ))
        );

        // public static final AprilTagFieldLayout FieldLayout =
        //         AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final AprilTagFieldLayout FieldLayout = new AprilTagFieldLayout(
            Tags,
            5,
            5
        );
    }

    public interface Modules{
        public static final double SpeedKP = 0.001, SpeedKI = 0, SpeedKD = 0.0005;
        public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
        
        public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;

        public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;

        public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;

        public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;

        SwerveModule[] moduleArray = new SwerveModule[] {
            new SwerveModule(FrontRightDriveID,FrontRightSteerID,FrontRightEncoderID, true),
            new SwerveModule(FrontLeftDriveID, FrontLeftSteerID, FrontLeftEncoderID, false),
            new SwerveModule(RearLeftDriveID, RearLeftSteerID, RearLeftEncoderID, false),
            new SwerveModule(RearRightDriveID, RearRightSteerID, RearRightEncoderID, true)
        };
        
    }

    public interface Indexer {
        public static final int BottomRollerID = 14;
        public static final int TopRollerID = 16;
    }

    public interface Shooter {
        public static final int MotorID = 17;
    }

    public interface Intake {
        public static final int ArmID = 15;
        public static final int RollerID = 13;
    }

    public interface Drivetrain {
        public interface Odometry {
            public static final double PositionStdDev = 0.1;
            public static final double AngleStdDev = 0.1;
        }

        public interface Vision {
            public static final double PositionStdDev = 1;
            // public static final double XConstantStdDev = 0.9;
            // public static final double XMagnitudeStdDev = 0.0;
            // public static final double YConstantStdDev = 0.9;
            // public static final double YMagnitudeStdDev = 0.0;
            // public static final double AngleStdDev = 0.1;

            // public static final double XRejectDistance = 1.0;
            // public static final double YRejectDistance = 1.0;

            // public static final double InitialTimeoutSeconds = 1.0;
            // public static final double TimeoutSeconds = 1.0;

            // public static final double TargetX = 0.0;
            // public static final double TargetY = -1.0;
            // public static final double TargetTheta = 0.0;
        }

		public static final double TranslationPow = 3;
        public static final double RotationPow = 3;

        public static final double SlowFactor = 3;
        public static final double SlowFactorOffset = 1;

        public static final double SteerMotorSlewRate = 20;
    }

    public interface Motion {
        public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
        public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
            public static final double xoffset = 0.5, yoffset = 0.25;
    }

}
