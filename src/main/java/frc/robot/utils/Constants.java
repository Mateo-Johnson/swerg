package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {

  public static CommandXboxController primary = new CommandXboxController(OIConstants.kDriverControllerPort);

  public static final class DriveConstants {
    // BE CAREFUL, THESE ARE THE MAX ALLOWED SPEEDS, NOT THE MAX CAPABLE
    public static final double kMaxSpeedMetersPerSecond = 5.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // RADIANS/SECOND

    // CHASSIS CONFIG
    //DISTANCE BETWEEN CENTER OF RIGHT AND LEFT MODULE 
    public static final double kTrackWidth = Units.inchesToMeters(23.75); // THIS IS CORRECT FOR 2025 DRIVETRAIN
    // DISTANCE BETWEEN CENTER OF FRONT AND BACK MODULE
    public static final double kWheelBase = Units.inchesToMeters(23.75);

// Example of completely inverted kinematics
public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  // front left - both X and Y flipped
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),   // front right - both X and Y flipped
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   // rear left - both X and Y flipped
    new Translation2d(kWheelBase / 2, kTrackWidth / 2)     // rear right - both X and Y flipped
);

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
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i GEAR RATIO CONFIG (I THINK)
    public static final double kDrivingMotorReduction = 6.12; // L3 RATIO
    public static final double kTurningMotorReduction = 21.4285714286; // MK4i RATIO

    // MOTOR ATTRIBUTES
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
      kDrivingMotorReduction;
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
    public static final int leftCoralID = 35;
    public static final int rightCoralID = 36;

    // CURRENT LIMITS
    public static final int SCL = 35; // SMART CURRENT LIMIT (CONT. CURRENT) IN AMPS
    public static final int FCL = 40; // FREE CURRENT LIMIT (PEAK CURRENT) IN AMPS

    // Game piece detection constants
    public static final double currentThreshold = 25.0; // Current threshold in amps to detect a game piece
    public static final double detectionTime = 0.1; // Time in seconds current must be above threshold
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

  public static final class PIDConstants {
    public static final PIDController translateController = new PIDController(0, 0, 0);
    public static final PIDController rotateController = new PIDController(0, 0, 0);
    public static PIDController xPID = new PIDController(0, 0, 0);
    public static PIDController yPID = new PIDController(1, 0, 0);
  }
}