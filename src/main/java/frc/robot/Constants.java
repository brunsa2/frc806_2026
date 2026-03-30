package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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

    public interface Pose {
        public static final String CameraName = "front";
        public static final Translation2d Position = new Translation2d(0, 0);

        // TODO: fix these fake values
        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 999999);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public interface Odometry {
            public static final double PositionStdDev = 0.1;
            public static final double AngleStdDev = 0.1;
        }

        public static final Transform3d RobotToCamera = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        );
        public static final AprilTagFieldLayout FieldLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    public interface Drivetrain {
        public static final double SpeedKP = 5, SpeedKI = 0, SpeedKD = 0;
        public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
        public static final double SteerDriveKP = 1.5, SteerDriveKI = 0, SteerDriveKD = 0;
        
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

        public static final double TranslationPow = 3;
        public static final double RotationPow = 3;

        public static final double SlowFactor = 3;
        public static final double SlowFactorOffset = 1;

        public static final double SteerMotorSlewRate = 20;
    }

    public interface Indexer {
        public static final int BottomRollerID = 14;
        public static final int TopRollerID = 16;
        public static final double floorIdleVoltage = 1.2;
        public static final double floorIndexVoltage = 4.8;
        public static final double ceilingIdleVoltage = 2.4;
        public static final double ceilingIndexVoltage = 8.4;
    }

    public interface Shooter {
        public static final int MotorID = 17;
        public static final double PrimeRPM = 3000;
        public static final double ShootRPM = 3500;
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.147368;
        public static final double kV = 0.105263;
    }

    public interface Intake {
        public static final int ArmID = 15;
        public static final int RollerID = 13;
        public static final int ArmEncoderId = 18;
        public static final double rollerVoltage = 12;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kP = 40;
        public static final double kI = 10;
        public static final double kD = 0;
        public static final double MotionMagicCruiseVelocity = 75;
        public static final double MotionMagicAcceleration = 2;
        public static final double ArmBottomPos = 0.08;
        public static final double ArmHorizontalPos = 0;
        public static final double ArmVerticalPos = -0.25;
        public static final double ArmBackPos = -0.28;
        
    }

    public interface Motion {
        public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
        public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
    }

}
