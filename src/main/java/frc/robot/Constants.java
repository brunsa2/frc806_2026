package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.SwerveModule;

public class Constants {

    public static final double Delta = 1e-2;
    
    // 25.25 inches wide, 18.16 inches deep
    public static final Translation2d[] moduleLocations = {
        new Translation2d(-0.320675, 0.230505),  //front right +-
        new Translation2d(-0.320675, -0.230505), //front left ++
        new Translation2d(0.320675, -0.230505),  //rear left -+
        new Translation2d(0.320675, 0.230505)  //rear right --
    };

    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = Math.sqrt(Math.pow(moduleLocations[1].getX(), 2) + Math.pow(moduleLocations[1].getY(), 2));
    public static final double attainableMaxModuleSpeedMPS = 4.572; // 15 feet/s
    public static final double attainableMaxTranslationalSpeedMPS = attainableMaxModuleSpeedMPS;
    public static final double attainableMaxRotationalVelocityRPS = attainableMaxModuleSpeedMPS/DriveBaseRadius;

    public static final int PigeonID = 22;   

    public static double controllerDeadband = 0.15; 

    public interface Modules{
        public static final double SpeedKP = 0.001, SpeedKI = 0, SpeedKD = 0.0005;
        public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
        
        public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;

        public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;

        public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;

        public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;

        SwerveModule[] moduleArray = new SwerveModule[] {
            new SwerveModule(FrontRightDriveID,FrontRightSteerID,FrontRightEncoderID),
            new SwerveModule(FrontLeftDriveID, FrontLeftSteerID, FrontLeftEncoderID),
            new SwerveModule(RearLeftDriveID, RearLeftSteerID, RearLeftEncoderID),
            new SwerveModule(RearRightDriveID, RearRightSteerID, RearRightEncoderID)
        };
        
    }

    public interface Drivetrain {
        public interface Odometry {
            public static final double PositionStdDev = 0.1;
            public static final double AngleStdDev = 0.1;
        }

        public interface Vision {
            public static final double XConstantStdDev = 0.9;
            public static final double XMagnitudeStdDev = 0.0;
            public static final double YConstantStdDev = 0.9;
            public static final double YMagnitudeStdDev = 0.0;
            public static final double AngleStdDev = 0.1;

            public static final double XRejectDistance = 1.0;
            public static final double YRejectDistance = 1.0;

            public static final double InitialTimeoutSeconds = 1.0;
            public static final double TimeoutSeconds = 1.0;

            public static final double TargetX = 0.0;
            public static final double TargetY = -1.0;
            public static final double TargetTheta = 0.0;
        }

		public static final double TranslationPow = 3;
        public static final double RotationPow = 3;

        public static final double SlowFactor = 3;
        public static final double SlowFactorOffset = 1;

        public static final double SteerMotorSlewRate = 100;
    }

    public interface Motion {
        public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
        public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
            public static final double xoffset = 0.5, yoffset = 0.25;
    }

}
