package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {


  public static final class DriveConstants {
    // BE CAREFUL, THESE ARE THE MAX ALLOWED SPEEDS, NOT THE MAX CAPABLE
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // RADIANS/SECOND

    // CHASSIS CONFIG
    //DISTANCE BETWEEN CENTER OF RIGHT AND LEFT MODULE 
    public static final double kTrackWidth = Units.inchesToMeters(23.75); // THIS IS CORRECT FOR 2025 DRIVETRAIN
    // DISTANCE BETWEEN CENTER OF FRONT AND BACK MODULE
    public static final double kWheelBase = Units.inchesToMeters(23.75);

    //CREATE A KINEMATICS OBJECT USING THOSE DIMENSIONS
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // SPARKMAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 6;

    // THRIFTY ENCODERS
    public static final int kFrontLeftEncoder = 1;
    public static final int kFrontRightEncoder = 0;
    public static final int kRearLeftEncoder = 2;
    public static final int kRearRightEncoder = 3;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i GEAR RATIO CONFIG (I THINK)
    public static final int kDrivingMotorPinionTeeth = 16; // STANDARD MK4i PINION
    public static final double kDrivingMotorReduction = 12.75; // L3 RATIO

    // MOTOR ATTRIBUTES
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; //CONTROLLER PORT
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // CONSTRAIN THE MOTION PROFILED CONTROLLER
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
