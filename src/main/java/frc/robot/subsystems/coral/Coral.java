package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CoralConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Counter.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coral extends SubsystemBase {
  // Hardware
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private AnalogInput sonarSensor;

  // Control
  private final PIDController leftVelocityPID;
  private final PIDController rightVelocityPID;
  private double targetVelocity = CoralConstants.targetVelocity;
  private IntakeMode currentMode = IntakeMode.STOPPED;
  private double lastDistance = 0;

  // Constants
  private static final int LEFT_MOTOR_ID = CoralConstants.leftCoralID;
  private static final int RIGHT_MOTOR_ID = CoralConstants.rightCoralID;
  private static final double LEFT_GEAR_RATIO = CoralConstants.leftGearRatio;
  private static final double RIGHT_GEAR_RATIO = CoralConstants.rightGearRatio;

  // MaxSonar Constants
  private static final double MAXSONAR_SCALE_FACTOR = 147.0; // microseconds per inch
  private static final double DISTANCE_THRESHOLD = 4.0; // inches

  // Speed Constants (in RPM at the roller)
  private static final double intake_speed = CoralConstants.intake_speed;
  private static final double fast_eject_speed = CoralConstants.fast_eject_speed;
  private static final double slow_eject_speed = CoralConstants.slow_eject_speed;
  private static final double hold_speed = CoralConstants.hold_speed;

  // PID Constants for velocity control
  private static final double kP = CoralConstants.kP;
  private static final double kI = CoralConstants.kI;
  private static final double kD = CoralConstants.kD;
  private static final double kF = CoralConstants.kF;

  // Current limiting
  private static final int SCL = CoralConstants.SCL;
  private static final int FCL = CoralConstants.FCL;

  public enum IntakeMode {
    INTAKING,
    FAST_EJECTING,
    SLOW_EJECTING,
    HOLDING,
    STOPPED
  }

  public Coral() {
    // Initialize hardware
    leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    // Initialize MaxSonar sensor
    sonarSensor = new AnalogInput(4);

    // Configure left motor
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(SCL, FCL);
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.inverted(false);
    leftConfig.encoder.velocityConversionFactor(1.0 / LEFT_GEAR_RATIO);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, null);

    // Configure right motor
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(SCL, FCL);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.inverted(true);
    rightConfig.encoder.velocityConversionFactor(1.0 / RIGHT_GEAR_RATIO);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, null);

    // Initialize PID Controllers
    leftVelocityPID = new PIDController(kP, kI, kD);
    rightVelocityPID = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {
    // Update sensor reading
    updateSensorReading();

    // Auto control based on sensor reading
    if (hasGamePiece()) {
      if (currentMode != IntakeMode.FAST_EJECTING) {
        fastEject();
      }
    } else {
      if (currentMode != IntakeMode.INTAKING) {
        intake();
      }
    }

    // Update motors
    double leftCurrentVelocity = leftEncoder.getVelocity();
    double rightCurrentVelocity = rightEncoder.getVelocity();

    double leftOutput = leftVelocityPID.calculate(leftCurrentVelocity, targetVelocity) +
      (targetVelocity * kF * (LEFT_GEAR_RATIO / RIGHT_GEAR_RATIO));
    double rightOutput = rightVelocityPID.calculate(rightCurrentVelocity, targetVelocity) +
      (targetVelocity * kF);

    leftMotor.set(leftOutput);
    rightMotor.set(rightOutput);

    // Log data
    SmartDashboard.putString("Intake Mode", currentMode.toString());
    SmartDashboard.putNumber("Left Intake Velocity", leftCurrentVelocity);
    SmartDashboard.putNumber("Right Intake Velocity", rightCurrentVelocity);
    SmartDashboard.putNumber("Intake Target Velocity", targetVelocity);
    SmartDashboard.putNumber("Distance to Game Piece (inches)", getDistance());
    SmartDashboard.putBoolean("Game Piece Detected", hasGamePiece());
    SmartDashboard.putNumber("Left Motor Current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Motor Current", rightMotor.getOutputCurrent());
  }

  private void updateSensorReading() {
    // Get the pulse width in microseconds and convert to distance
    double voltage = sonarSensor.getVoltage();
    lastDistance = voltage / 0.00977; // DISTANCE CM
  }

  public double getDistance() {
    return lastDistance;
  }

  public boolean hasGamePiece() {
    return getDistance() < DISTANCE_THRESHOLD;
  }

  // Control Methods
  public void intake() {
    setMode(IntakeMode.INTAKING);
  }

  public void fastEject() {
    setMode(IntakeMode.FAST_EJECTING);
  }

  public void slowEject() {
    setMode(IntakeMode.SLOW_EJECTING);
  }

  public void hold() {
    setMode(IntakeMode.HOLDING);
  }

  public void stop() {
    setMode(IntakeMode.STOPPED);
  }

  private void setMode(IntakeMode mode) {
    currentMode = mode;
    switch (mode) {
    case INTAKING:
      targetVelocity = intake_speed;
      break;
    case FAST_EJECTING:
      targetVelocity = fast_eject_speed;
      break;
    case SLOW_EJECTING:
      targetVelocity = slow_eject_speed;
      break;
    case HOLDING:
      targetVelocity = hold_speed;
      break;
    case STOPPED:
    default:
      targetVelocity = 0.0;
      break;
    }
  }

  // Getter Methods
  public IntakeMode getCurrentMode() {
    return currentMode;
  }

  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftCurrentDraw() {
    return leftMotor.getOutputCurrent();
  }

  public double getRightCurrentDraw() {
    return rightMotor.getOutputCurrent();
  }

  public boolean isAtTargetVelocity() {
    return Math.abs(leftEncoder.getVelocity() - targetVelocity) < CoralConstants.velocityTolerance &&
      Math.abs(rightEncoder.getVelocity() - targetVelocity) < CoralConstants.velocityTolerance;
  }
}