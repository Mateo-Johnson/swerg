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
    // Be careful, these are the max allowed speeds, not the max capable
    public static final double kMaxSpeedMetersPerSecond = 5.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians/second

    // Chassis Config

    // Distance between the center of right and left modules 
    public static final double kTrackWidth = Units.inchesToMeters(23.75); // This is correct for 2025 drivetrain
    // Distance between the center of the front and back modules
    public static final double kWheelBase = Units.inchesToMeters(23.75);

    // IF IT DOES NOT WORK REVERT THE KINEMATICS
    // public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), 
    //     new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  
    //     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   
    //     new Translation2d(kWheelBase / 2, kTrackWidth / 2)     
    // );
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Rear Left
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Rear Right

    // SparkMax CAN IDs

    // Driving
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 5;

    // Turning
    public static final int kFrontLeftTurningCanId = 9;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 6;

    // Encoders
    public static final int kFrontLeftEncoder = 1;
    public static final int kFrontRightEncoder = 0;
    public static final int kRearLeftEncoder = 2;
    public static final int kRearRightEncoder = 3;
  }

  public static final class ModuleConstants {
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // MK4i gear ratio
    public static final double kDrivingMotorReduction = 6.12; // L3 ratio
    public static final double kTurningMotorReduction = 21.4285714286; // MK4i ratio

    // Motor attributes
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
      kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Controller port
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {

    public static final double centerSetpoint = 0;
    public static final double leftSetpoint = 0.22;
    public static final double rightSetpoint = -0.12;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constrain the motion profiled controller
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

    // Current limits
    public static final int SCL = 35; // Smart current limit (cont. current) in amps
    public static final int FCL = 40; // Free current limit (peak current) in amps

    public static final double currentThreshold = 8.0; // Threshold for game piece detection (Amps)
  }

  public static final class ElevatorConstants {
    public static final int leftCANId = 11;
    public static final int rightCANId = 12;
    public static final int limitSwitchPort = 0;

    public static final double gravityCompensation = 0.05;
    public static final double maxManualSpeed = 0.7;
    public static final double deadband = 0.1;
    public static final double tolerance = 0.05;
    public static final int maxHeight = 50;
  }

  public static final class AlgaeConstants {
    public static final int algaeIntakeCANId = 31;
    public static final int algaePivotCANId = 23;

  }

  public static final class PIDConstants {
    public static final PIDController translateController = new PIDController(0, 0, 0);
    public static final PIDController rotateController = new PIDController(0, 0, 0);
    public static PIDController xPID = new PIDController(0, 0, 0);
    public static PIDController yPID = new PIDController(0.3, 0.01, 0);
  }
}