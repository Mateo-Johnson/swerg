package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Preferences;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class Module {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final AnalogInput m_turningAnalogEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private double m_analogEncoderOffset;
  private final String m_moduleName;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private double m_lastPosition = 0.0;
  private final LinearFilter m_voltageFilter = LinearFilter.movingAverage(10);

  /**
   * Constructs a MAXSwerveModule with a Thrifty absolute encoder
   */
  public Module(String moduleName, int drivingCANId, int turningCANId, int analogPort, double chassisAngularOffset) {
    m_moduleName = moduleName;
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    m_turningAnalogEncoder = new AnalogInput(analogPort);
    m_turningAnalogEncoder.setAverageBits(4);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Load the stored offset, defaulting to 0 if not calibrated
    m_analogEncoderOffset = Preferences.getDouble(m_moduleName + "_offset", 0.0);

    // Apply configurations
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    
    // Initialize the relative encoder to match the absolute position
    m_lastPosition = getAbsoluteEncoderRadians();
    m_turningEncoder.setPosition(m_lastPosition);
  }

  /**
   * Calibrates the absolute encoder offset.
   * Call this method when all modules are physically set to their zero position.
   */
  public void calibrateOffset() {
    // Get the current raw position
    double rawPosition = getRawEncoderRadians();
    
    // Calculate the new offset that would make this position read as zero
    m_analogEncoderOffset = rawPosition;
    
    // Save to persistent storage
    Preferences.setDouble(m_moduleName + "_offset", m_analogEncoderOffset);
  }

  /**
   * Gets the raw encoder reading without any offset applied
   */
  private double getRawEncoderRadians() {
    double voltage = m_voltageFilter.calculate(m_turningAnalogEncoder.getAverageVoltage());
    double position = voltage * Configs.MAXSwerveModule.THRIFTY_VOLTAGE_TO_RADIANS;
    
    // Normalize to [0, 2π]
    position %= 2.0 * Math.PI;
    if (position < 0.0) {
      position += 2.0 * Math.PI;
    }
    
    return position;
  }

  /**
   * Gets the absolute encoder angle in radians with filtering and offset
   */
  private double getAbsoluteEncoderRadians() {
    double position = getRawEncoderRadians();
    position -= m_analogEncoderOffset;
    
    // Normalize again after applying offset
    position %= 2.0 * Math.PI;
    if (position < 0.0) {
      position += 2.0 * Math.PI;
    }
    
    // Apply additional filtering between readings
    position = m_lastPosition + Configs.MAXSwerveModule.THRIFTY_FILTER_ALPHA * (position - m_lastPosition);
    m_lastPosition = position;
    
    return position;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(getAbsoluteEncoderRadians() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getAbsoluteEncoderRadians() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Get the current angle for optimization
    double currentAngleRadians = getAbsoluteEncoderRadians();
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(currentAngleRadians));

    // Command driving and turning SPARKS towards their respective setpoints
    m_drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    m_lastPosition = getAbsoluteEncoderRadians();
    m_turningEncoder.setPosition(m_lastPosition);
  }
}