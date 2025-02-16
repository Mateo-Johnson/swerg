package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.OIConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {   
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                // LEFT STICK CONTROLS TRANSLATION
                // RIGHT STICK (LEFT/RIGHT) CONTROL TURNING
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}

//EXAMPLE AUTO COMMAND

// // Create config for trajectory
// TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(DriveConstants.kDriveKinematics);

// // An example trajectory to follow. All units in meters.
// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(3, 0, new Rotation2d(0)),
//         config);

// var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
// thetaController.enableContinuousInput(-Math.PI, Math.PI);

// SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         m_robotDrive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         m_robotDrive::setModuleStates,
//         m_robotDrive);

// // Reset odometry to the starting pose of the trajectory.
// m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

// // Run path following command, then stop at the end.
// return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
