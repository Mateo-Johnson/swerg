package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
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
        // Get raw position (0-1)
        double pos = m_turningAnalogEncoder.get();
        
        // Convert to full rotation
        double position = pos * (2 * Math.PI);
        
        // Apply offset before normalization
        position = position - m_analogEncoderOffset;
        
        // Use MathUtil to normalize to [-π, π]
        return MathUtil.angleModulus(position);
    }

    public double getRawRadians() {
        // Get position 0-1
        double pos = m_turningAnalogEncoder.get();
        // Convert to position
        double position = pos * (2 * Math.PI);
        // Normalize to [-π, π]
        position = position % (2 * Math.PI);
        if (position > Math.PI) {
            position -= 2 * Math.PI;
        } else if (position < -Math.PI) {
            position += 2 * Math.PI;
        }
        return position;
    }

    public double getRawAngle() {
        double raw = m_turningAnalogEncoder.get();
        return raw;
    }

    private void syncEncoders() {
        // Get raw angle without offset
        double angle = getAngle();
    
        // Disable closed-loop control temporarily
        m_turningClosedLoopController.setReference(0, ControlType.kDutyCycle);
    
        // Set relative encoder position to match absolute position
        m_turningEncoder.setPosition(getAngle());
    
        // Apply offset and normalize exactly like getAngle() does
        double targetAngle = angle;
        
        m_turningClosedLoopController.setReference(targetAngle, ControlType.kPosition);
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

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    
        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    
        m_desiredState = desiredState;
      }

    public void resetEncoders() {
        syncEncoders();
    }

    
    
}