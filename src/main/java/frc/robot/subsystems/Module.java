package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Module {
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final AnalogEncoder m_turningAnalogEncoder;
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
    private final double m_analogEncoderOffset;
    @SuppressWarnings("unused")
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public Module(int drivingCANId, int turningCANId, int analogPort, double analogOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getEncoder();
        m_turningAnalogEncoder = new AnalogEncoder(analogPort);
        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
        m_analogEncoderOffset = analogOffset;

        // Apply configurations
        m_drivingSpark.configure(Config.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Config.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        syncEncoders();
    }

    public double getAngle() {
        // Get position 0-1
        double pos = m_turningAnalogEncoder.get();
        // Convert to position
        double position = pos * (2*Math.PI);
        position = position - m_analogEncoderOffset;
        // Normalize to [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }
        return position;
    }

    public double getAngleDegrees() {
        // Get position [0, 1]
        double pos = m_turningAnalogEncoder.get();
        // Convert to position [0, 2π]
        double position = pos * (2*Math.PI);
        //Subtract Angular Offset
        position = position - m_analogEncoderOffset;
        // Normalize to [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }
        //Convert to degrees
        return Math.toDegrees(position);
    }

    public double getRawRadians() {
        // Get position 0-1
        double pos = m_turningAnalogEncoder.get();
        // Convert to position
        double position = pos * (2*Math.PI);
        // Normalize to [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }
        return position;
    }

    public double getRawAngle() {
        double raw = m_turningAnalogEncoder.get();
        return raw;
    }

    private void syncEncoders() {
        // Get current absolute angle
        double absoluteAngle = getAngle();

        // Disable closed-loop control temporarily
        m_turningClosedLoopController.setReference(0, ControlType.kDutyCycle);

        // Set relative encoder position to match absolute position
        m_turningEncoder.setPosition(absoluteAngle);

        // Re-enable closed-loop control
        m_turningClosedLoopController.setReference(absoluteAngle, ControlType.kPosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_drivingEncoder.getPosition(),
                new Rotation2d(getAngle() ));
    }

    public void setDesiredState(SwerveModuleState desiredState, int moduleNumber) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(30));

    // Get the current angle for optimization
    double currentAngleRadians = getAngle();

    // Optimize the reference state
    @SuppressWarnings("deprecation")
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(currentAngleRadians));

    // Command driving and turning SPARKS
    m_drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
    
    // Store the optimized state instead of the original
    m_desiredState = optimizedDesiredState;
}

    public void resetEncoders() {
        syncEncoders();
    }
}