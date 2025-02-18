package frc.robot.utils;

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
    // DRIVING
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 5;

    // TURNING
    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 6;

    // THRIFTY ENCODERS
    public static final int kFrontLeftEncoder = 1;
    public static final int kFrontRightEncoder = 0;
    public static final int kRearLeftEncoder = 2;
    public static final int kRearRightEncoder = 3;

    //IS THE GYRO REVERSED
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i GEAR RATIO CONFIG (I THINK)
    public static final double kDrivingMotorReduction = 12.75; // L3 RATIO
    public static final double kTurningMotorReduction = 21.43;

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

  public static final class CoralConstants {
    // Motor IDs
    public static final int leftCoralID = 21;  // Update with actual CAN ID
    public static final int rightCoralID = 22; // Update with actual CAN ID
    public static final int intakeLimitSwitchPort = 1;

    // Mechanical Configuration
    public static final double leftGearRatio = 5.0;  // Update with actual left gear ratio
    public static final double rightGearRatio = 7.0; // Update with actual right gear ratio
    
    // Speed Constants (in RPM at the roller)
    public static final double targetVelocity = 0.0;  // Default target velocity
    public static final double intake_speed = 3000.0; // Positive for intaking
    public static final double fast_eject_speed = -3000.0; // Negative for ejecting
    public static final double slow_eject_speed = -1500.0; // Half speed eject
    public static final double hold_speed = 500.0;    // Low speed to hold game piece
    
    // PID Constants
    public static final double kP = 0.0001;  // Proportional gain
    public static final double kI = 0.0;     // Integral gain
    public static final double kD = 0.0;     // Derivative gain
    public static final double kF = 0.00017; // Feedforward gain
    
    // Current Limiting
    public static final int SCL = 35; // Smart Current Limit (sustained current) in amps
    public static final int FCL = 40; // Free Current Limit (peak current) in amps
    
    // Tuning Constants
    public static final double velocityTolerance = 50.0; // RPM tolerance for isAtTargetVelocity
  }

  public static final class ElevatorConstants {
    public static final int leftCANId = 11;
    public static final int rightCANId = 12;
    public static final int limitSwitchPort = 0;
  }

  public static final class AlgaeConstants {
    public static final int algaeIntakeCANId = 31;
    public static final int algaePivotCANId = 32;

  }
}
