package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        
        // Define separate encoder instances for each module
        private static final AnalogEncoder frontLeftEncoder = new AnalogEncoder(
            DriveConstants.kFrontLeftEncoder,
            2 * Math.PI,
            Math.PI
        );
        
        private static final AnalogEncoder frontRightEncoder = new AnalogEncoder(
            DriveConstants.kFrontRightEncoder,
            2 * Math.PI,
            Math.PI
        );
        
        private static final AnalogEncoder rearLeftEncoder = new AnalogEncoder(
            DriveConstants.kRearLeftEncoder,
            2 * Math.PI,
            Math.PI
        );
        
        private static final AnalogEncoder rearRightEncoder = new AnalogEncoder(
                DriveConstants.kRearRightEncoder,
            2 * Math.PI,
            Math.PI
        );

        static {
            // Driving configuration remains unchanged
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            // Turning configuration modified for external analog encoder
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            
            // Configure the turning motor to use the external analog encoder
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}