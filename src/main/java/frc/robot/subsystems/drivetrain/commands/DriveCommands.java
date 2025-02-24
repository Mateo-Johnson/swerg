package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * A collection of commands for the Drivetrain subsystem, which controls robot movement.
 */
public class DriveCommands {
  // PID Constants for movement
  private static final double drive_kP = 0.0000001;
  private static final double drive_kI = 0.0;
  private static final double drive_kD = 0.0;
  private static final double drive_kTolerance = 0.02; // 2 cm tolerance

  // PID Constants for movement
  private static final double turn_kP = 0.0000001;
  private static final double turn_kI = 0.0;
  private static final double turn_kD = 0.0;
  private static final double turn_kTolerance = 0.02;

  /**
   * Creates a command that moves the robot along the X axis.
   * 
   * @param drivetrain The Drivetrain subsystem to be used.
   * @param targetX    The target X coordinate in meters.
   * @return The Command that will move the robot along the X axis.
   */
  public static Command translateX(Drivetrain drivetrain, double targetX) {
    return new Command() {
      private final PIDController xTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);
      private final PIDController yTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);

      @Override
      public void initialize() {
        xTranslatePID.setTolerance(drive_kTolerance);
        yTranslatePID.setTolerance(drive_kTolerance);
        xTranslatePID.reset();
        yTranslatePID.reset();
        addRequirements(drivetrain);
      }

      @Override
      public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        double xOutput = xTranslatePID.calculate(currentPose.getX(), targetX);
        double yOutput = yTranslatePID.calculate(currentPose.getY(), currentPose.getY());
        drivetrain.drive(xOutput, yOutput, 0, true);
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
      }

      @Override
      public boolean isFinished() {
        return xTranslatePID.atSetpoint();
      }
    };
  }

  /**
   * Creates a command that moves the robot along the Y axis.
   * 
   * @param drivetrain The Drivetrain subsystem to be used.
   * @param targetY    The target Y coordinate in meters.
   * @return The Command that will move the robot along the Y axis.
   */
  public static Command translateY(Drivetrain drivetrain, double targetY) {
    return new Command() {
      private final PIDController xTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);
      private final PIDController yTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);

      @Override
      public void initialize() {
        xTranslatePID.setTolerance(drive_kTolerance);
        yTranslatePID.setTolerance(drive_kTolerance);
        xTranslatePID.reset();
        yTranslatePID.reset();
        addRequirements(drivetrain);
      }

      @Override
      public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        double xOutput = xTranslatePID.calculate(currentPose.getX(), currentPose.getX());
        double yOutput = yTranslatePID.calculate(currentPose.getY(), targetY);
        drivetrain.drive(xOutput, yOutput, 0, true);
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
      }

      @Override
      public boolean isFinished() {
        return yTranslatePID.atSetpoint();
      }
    };
  }

  /**
   * Creates a command that moves the robot to a target position.
   * 
   * @param drivetrain The Drivetrain subsystem to be used.
   * @param targetX    The target X coordinate in meters.
   * @param targetY    The target Y coordinate in meters.
   * @return The Command that will move the robot to the target position.
   */
  public static Command translate(Drivetrain drivetrain, double targetX, double targetY) {
    return new Command() {
      private final PIDController xTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);
      private final PIDController yTranslatePID = new PIDController(drive_kP, drive_kI, drive_kD);

      @Override
      public void initialize() {
        xTranslatePID.setTolerance(drive_kTolerance);
        yTranslatePID.setTolerance(drive_kTolerance);
        xTranslatePID.reset();
        yTranslatePID.reset();
        addRequirements(drivetrain);
      }

      @Override
      public void execute() {
        Pose2d currentPose = drivetrain.getPose();
        double xOutput = xTranslatePID.calculate(currentPose.getX(), targetX);
        double yOutput = yTranslatePID.calculate(currentPose.getY(), targetY);
        drivetrain.drive(xOutput, yOutput, 0, true);
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
      }

      @Override
      public boolean isFinished() {
        return xTranslatePID.atSetpoint() && yTranslatePID.atSetpoint();
      }
    };
  }

  /**
   * Creates a command that turns the robot to a target angle.
   * 
   * @param drivetrain The Drivetrain subsystem to be used.
   * @param targetAngle The target angle in degrees.
   * @return The Command that will turn the robot to the target angle.
   */
  public static Command rotate(Drivetrain drivetrain, int targetAngle) {
    return new Command() {
      private final PIDController turnPID = new PIDController(turn_kP, turn_kI, turn_kD);
      private double currentAngle;

      @Override
      public void initialize() {
        turnPID.setTolerance(turn_kTolerance);
        addRequirements(drivetrain);
      }

      @Override
      public void execute() {
        currentAngle = drivetrain.getHeading();
        drivetrain.drive(0, 0, turnPID.calculate(currentAngle, targetAngle), true);
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
      }

      @Override
      public boolean isFinished() {
        return Math.abs(currentAngle - targetAngle) < 0.5;
      }
    };
  }

  public static Command zeroWheels(Drivetrain drivetrain) {
    return new Command() {
      @Override
      public void initialize() {
        // Reset all modules to zero speed and zero angle
        drivetrain.resetWheels();
      }

      @Override
      public boolean isFinished() {
        return true; // Command completes immediately
      }
    };
  }
}