package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.commands.L3_Remove;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.coral.commands.Purge;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.Align;
import frc.robot.subsystems.drivetrain.commands.AlignLeft;
import frc.robot.subsystems.drivetrain.commands.AlignRight;
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
  // Auto SendableChooser
  private final SendableChooser<Command> autoChooser;
  // Subsystem declarations
  final Drivetrain m_drivetrain;
  final Elevator m_elevator;
  final Coral m_coral;
  final Algae m_algae;
  
  // The driver's controller
  private final CommandXboxController primary = Constants.primary;

   public RobotContainer() {

    m_elevator = new Elevator();
    m_drivetrain = new Drivetrain(m_elevator);
    m_coral = new Coral();
    m_algae = new Algae();
    
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    // Specify default auto by name autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

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

  SmartDashboard.putData("Auto Chooser", autoChooser); // Put the auto chooser on the dashboard
}

  private void registerNamedCommands() {
    NamedCommands.registerCommand("L1", new MoveToPoint(m_elevator, 0).withTimeout(1));
    NamedCommands.registerCommand("L2", new MoveToPoint(m_elevator, 25).withTimeout(1));
    NamedCommands.registerCommand("L3", new MoveToPoint(m_elevator, 25).withTimeout(1));
    NamedCommands.registerCommand("L4", new MoveToPoint(m_elevator, 25).withTimeout(3));
    NamedCommands.registerCommand("DownManual", new MoveManual(m_elevator, -0.1));
    NamedCommands.registerCommand("UpManual", new MoveManual(m_elevator, 0.2));

    NamedCommands.registerCommand("Intake", new Intake(m_coral, 0.7).withTimeout(0.5));
    NamedCommands.registerCommand("Purge", new Purge(m_coral, 0.5));
    
    NamedCommands.registerCommand("AlignLeft", new AlignLeft(m_drivetrain));
    NamedCommands.registerCommand("AlignRight", new AlignRight(m_drivetrain));
  }

  private void configureButtonBindings() {

    // Elevator Heights
    // L1 = 0
    // L2 = 5
    // L3 = 17.5
    // L4 = 42 ish

    // Drivetrain Commands
    primary.a().toggleOnTrue(new Align(m_drivetrain)); // Align the robot to the reef apriltags

    // Elevator Commands
    primary.rightBumper().whileTrue(new MoveManual(m_elevator, 0.2)); // Right bumper to move elevator up 
    primary.leftBumper().whileTrue(new MoveManual(m_elevator, -0.1)); // Left bumper to move elevator down
    //primary.povLeft().onTrue(new MoveToPoint(m_elevator, 17.5, new Intake(m_coral, 0.7), 0.5));
    primary.povLeft().onTrue(new MoveToPoint(m_elevator, 17.5, new Intake(m_coral, 0.7)));
    primary.povRight().onTrue(new MoveToPoint(m_elevator, 5, new Intake(m_coral, 0.7)));
    primary.povUp().onTrue(new L4(m_elevator, m_coral)); // Custom L4 command to execute the routine
    primary.povDown().onTrue(new MoveToPoint(m_elevator, 0)); // Custom L1 command to spin it sideways

    // Coral Commands
    // primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // Right trigger to intake coral
    primary.leftTrigger().whileTrue(new Purge(m_coral, 0.5)); // Left trigger to purge coral
    primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // Right trigger to intake coral (auto-stop)

    primary.y().whileTrue(new L3_Remove(m_elevator, m_algae));
    primary.b().whileTrue(new L3_Remove(m_elevator, m_algae));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
