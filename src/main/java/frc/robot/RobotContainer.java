package frc.robot;

// import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.coral.commands.Outtake;
import frc.robot.subsystems.coral.commands.Purge;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveManual;
import frc.robot.subsystems.elevator.commands.MoveToPoint;
import frc.robot.subsystems.vision.commands.general.AlignY;
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


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


   public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS
        new RunCommand(() -> {
            m_drivetrain.drive(
                MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband),
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
    primary.rightBumper().whileTrue(new MoveManual(m_elevator, 0.2)); // RIGHT BUMPER TO MOVE ELEVATOR UP
    primary.leftBumper().whileTrue(new MoveManual(m_elevator, -0.1)); // LEFT BUMPER TO MOVE ELEVATOR DOWN
    primary.povLeft().onTrue(new MoveToPoint(m_elevator, 22));
    primary.povDown().onTrue(new MoveToPoint(m_elevator, 7.5));

    // CORAL COMMANDS
    // primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // RIGHT TRIGGER TO INTAKE CORAL
    primary.leftTrigger().whileTrue(new Purge(m_coral, 0.5)); // LEFT TRIGGER TO PURGE CORAL

    primary.a().whileTrue(new AlignY(0, m_drivetrain));

    //THIS IS A TRIGGER COMMAND THAT CHECKS IF THE RIGHT TRIGGER IS PRESSED AND IF B IS PRESSED
    //IF BOTH ARE TRUE, IT WILL RUN THE COMMAND INSIDE THE WHILETRUE
    //WHEN THE TRIGGER IS RELEASED OR B IS RELEASED, THE COMMAND WILL STOP

    //THIS COMMAND WILL RUN THE OUTTAKE COMMAND WITH THE CURRENT TRIGGER VALUE
    new Trigger(() -> 
    primary.getRightTriggerAxis() > 0.1 && primary.b().getAsBoolean())
    .whileTrue(new RunCommand(() -> {
      // Create and schedule a new Outtake command with the current trigger value
      double speed = primary.getRightTriggerAxis();
      new Outtake(m_coral, speed).schedule();
    }, m_coral)
  );

  // THIS COMMAND WILL RUN THE INTAKE COMMAND WITH THE CURRENT TRIGGER VALUE
  new Trigger(() -> 
    primary.getRightTriggerAxis() > 0.1 && !primary.b().getAsBoolean())
    .whileTrue(new Intake(m_coral, 0.7)); 

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
