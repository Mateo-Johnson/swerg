package frc.robot;

// import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.coral.commands.Outtake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveManual;
import frc.robot.subsystems.elevator.commands.MoveToPoint;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  private final Coral m_coral = new Coral();


  // The driver's controller
  private final CommandXboxController primary = Constants.primary;
  private boolean slowMode = false; // Variable to track slow mode state


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


   public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS
        new RunCommand(() -> {
            double speedModifier = slowMode ? 0.5 : 1.0; // Reduce speed if slow mode is on
            m_drivetrain.drive(
                MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband) * speedModifier,
                MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband) * speedModifier,
                MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband) * speedModifier,
                true);
        }, m_drivetrain)
    );


    // COMMAND THE ELEVATOR TO HOLD IF NOT DOING ANYTHING ELSE
    // m_elevator.setDefaultCommand(ElevatorCommands.elevatorHold(m_elevator));


    // COMMAND THE CORAL INTAKE TO STORE WHEN NOT DOING ANYTHING ELSE
    // m_coral.setDefaultCommand(CoralCommands.hold(m_coral));
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


    // ELEVATOR COMMANDS
    primary.rightBumper().whileTrue(new MoveManual(m_elevator, 0.1)); // RIGHT BUMPER TO MOVE ELEVATOR UP
    primary.leftBumper().whileTrue(new MoveManual(m_elevator, -0.1)); // LEFT BUMPER TO MOVE ELEVATOR UP
    primary.povLeft().onTrue(new MoveToPoint(m_elevator, 21));
    primary.povDown().onTrue(new MoveToPoint(m_elevator, 7));
   
    // primary.povUp().whileTrue(ElevatorCommands.moveToL4(m_elevator));
    // primary.povRight().whileTrue(ElevatorCommands.moveToL3(m_elevator));
    // primary.povDown().whileTrue(ElevatorCommands.moveToL2(m_elevator));
    // primary.povLeft().whileTrue(ElevatorCommands.moveToL1(m_elevator));

    // CORAL COMMANDS
    primary.rightTrigger().whileTrue(new Intake(m_coral, 0.5)); // RIGHT TRIGGER TO INTAKE CORAL
    primary.leftTrigger().whileTrue(new Outtake(m_coral, 0.5)); // LEFT TRIGGER TO OUTTAKE CORAL

    // ALGAE COMMANDS
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("Example Auto");
    return null;
  }
}
