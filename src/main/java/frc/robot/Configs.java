package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class MAXSwerveModule {
            public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
            public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    
            static {
                // Driving configuration
                double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
                double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
                drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);
    
                // Turning configuration
                turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            }
        }
    }