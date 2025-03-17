package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.coral.commands.Purge;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.AlignY;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.L4;
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
  // Subsystem declarations
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  private final Coral m_coral = new Coral();
  // private final Algae m_algae = new Algae();
  
  // The driver's controller
  private final CommandXboxController primary = Constants.primary;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   public RobotContainer() {

    // Pathplanner named command declarations
    NamedCommands.registerCommand("MoveToPoint",  new MoveToPoint(m_elevator, 25));
    NamedCommands.registerCommand("Intake", new Intake(m_coral, 0.7));
    NamedCommands.registerCommand("Purge", new Purge(m_coral, 0.5));
    NamedCommands.registerCommand("AlignY", new AlignY(m_drivetrain));
    NamedCommands.registerCommand("DownManual", new MoveManual(m_elevator, -0.1));
    NamedCommands.registerCommand("UpManual", new MoveManual(m_elevator, 0.2));

    // Configure the button bindings
    configureButtonBindings();


    m_drivetrain.setDefaultCommand( // IF THE DRIVETRAIN ISN'T DOING ANYTHING ELSE, DO THIS
        new RunCommand(() -> {
            m_drivetrain.drive(
                MathUtil.applyDeadband(primary.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(primary.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(primary.getRightX(), OIConstants.kDriveDeadband),
                true);
        }, m_drivetrain)
    );
}

  private void configureButtonBindings() {

    // Elevator Heights
    // L1 = 0
    // L2 = 5
    // L3 = 17.5
    // L4 = UNKNOWN

    // Drivetrain Commands
    primary.a().toggleOnTrue(new AlignY(m_drivetrain)); 

    // Elevator Commands
    primary.rightBumper().whileTrue(new MoveManual(m_elevator, 0.2)); // Right bumper to move elevator up 
    primary.leftBumper().whileTrue(new MoveManual(m_elevator, -0.1)); // Left bumper to move elevator down
    //primary.povLeft().onTrue(new MoveToPoint(m_elevator, 17.5, new Intake(m_coral, 0.7), 0.5));
    primary.povLeft().onTrue(new MoveToPoint(m_elevator, 17.5, new Intake(m_coral, 0.7)));
    primary.povRight().onTrue(new MoveToPoint(m_elevator, 5, new Intake(m_coral, 0.7)));
    primary.povUp().onTrue(new L4(m_elevator, m_coral));
    primary.povDown().onTrue(new MoveToPoint(m_elevator, 0, new Intake(m_coral, 0.7)));

    // Coral Commands
    // primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // Right trigger to intake coral
    primary.leftTrigger().whileTrue(new Purge(m_coral, 0.5)); // Left trigger to purge coral
    primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // Right trigger to intake coral (auto-stop)
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("mick");
  }
}
