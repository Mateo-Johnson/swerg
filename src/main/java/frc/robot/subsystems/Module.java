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
    //SPARKS, ENCODERS, CLOSED LOOP CONTROLLERS
    private final SparkMax m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    //ANALOG ENCODER/OFFSET
    private final AnalogEncoder m_turningAnalogEncoder;
    private final double m_analogEncoderOffset;
    
    @SuppressWarnings("unused")
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    //CONSTRUCTOR FOR MODULE
    public Module(int drivingCANId, int turningCANId, int analogPort, double analogOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getEncoder();
        m_turningAnalogEncoder = new AnalogEncoder(analogPort);
        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();
        m_analogEncoderOffset = analogOffset;

        // APPLY CONFIG
        m_drivingSpark.configure(Config.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turningSpark.configure(Config.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        syncEncoders();
    }

    public double getAngle() {

        // GET POSITION [0, 1]
        double pos = m_turningAnalogEncoder.get();

        // CONVERT TO RADIANS
        double position = pos * (2*Math.PI);

        // SUBRACT OFFSET
        position = position - m_analogEncoderOffset;

        // NORMALIZE TO [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }

        return position;
    }

    public double getAngleDegrees() {

        // GET POSITION [0, 1]
        double pos = m_turningAnalogEncoder.get();

        // CONVERT TO RADIANS AND THEN DEGREES
        double position = Math.toDegrees(pos * (2*Math.PI));

        // SUBTRACT OFFSET
        position = position - m_analogEncoderOffset;

        // NORMALIZE TO [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }

        return position;
    }

    public double getRawRadians() {

        // GET POSITION [0, 1]
        double pos = m_turningAnalogEncoder.get();

        // CONVERT TO RADIANS
        double position = pos * (2*Math.PI);

        // NORMALIZE TO [0, 2π]
        position %= 2.0 * Math.PI;
        if (position < 0.0) {
            position += 2.0 * Math.PI;
        }

        //DO NOT SUBTRACT ANGULAR OFFSET

        return position;
    }

    public double getRawAngle() {
        // GET POSITION [0, 1]
        double raw = m_turningAnalogEncoder.get();
        return raw;
    }

    private void syncEncoders() {
        // GET CURRENT ANGLE WITH OFFSET
        double absoluteAngle = getAngle();

        // DISABLE CLOSED LOOP CONTROL TEMPORARILY
        m_turningClosedLoopController.setReference(0, ControlType.kDutyCycle);

        // SET RELATIVE ENCODER POSITION TO MATCH ABSOLUTE ENCODER POSITION
        m_turningEncoder.setPosition(absoluteAngle);

        // RE-ENABLE CLOSED LOOP CONTROL
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

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        // GET THE CURRENT ANGLE TO OPTIMIZE
        double currentAngleRadians = getAngle();

        // OPTIMIZE TO STOP FROM SPINNING MORE THAN 90°
        @SuppressWarnings("deprecation")
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(currentAngleRadians));

        // COMMAND DRIVING AND TURNING SPARKS TOWARD SETPOINT
        m_drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        syncEncoders();
    }
}