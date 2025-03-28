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
import frc.robot.subsystems.algae.commands.L2_Remove;
import frc.robot.subsystems.algae.commands.L3_Remove;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.coral.commands.Purge;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.Align;
import frc.robot.subsystems.drivetrain.commands.AlignLeft;
import frc.robot.subsystems.drivetrain.commands.Forward;
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
    NamedCommands.registerCommand("AlignLeft", new AlignLeft(m_drivetrain).withTimeout(1.5));
    NamedCommands.registerCommand("AlignForward", new Forward(m_drivetrain).withTimeout(2));
    NamedCommands.registerCommand("L3", new MoveToPoint(m_elevator, 43, new Intake(m_coral, 0.3)));
    NamedCommands.registerCommand("L4", new Intake(m_coral, 0.2));

  }

  private void configureButtonBindings() {

    // Elevator Heights
    // L1 = 0
    // L2 = 5
    // L3 = 17.5
    // L4 = 42 ish

    // Drivetrain Commands - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    primary.a().toggleOnTrue(new Align(m_drivetrain)); // Align the robot to the reef apriltags

    // Elevator Commands - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    primary.rightBumper().whileTrue(new MoveManual(m_elevator, 0.2)); // Right bumper to move elevator up 
    primary.leftBumper().whileTrue(new MoveManual(m_elevator, -0.1)); // Left bumper to move elevator down

    // L1-L4 commands with intake
    primary.povDown().onTrue(new MoveToPoint(m_elevator, 0)); // L1
    primary.povRight().onTrue(new MoveToPoint(m_elevator, 5)); // L2
    primary.povLeft().onTrue(new MoveToPoint(m_elevator, 19.5, new Intake(m_coral, 0.3))); // L3
    primary.povUp().onTrue(new MoveToPoint(m_elevator, 43, new Intake(m_coral, 0.3))); // L4

    // Commands with automatic outtake
    // primary.povDown().onTrue(new MoveToPoint(m_elevator, 0)); // L1
    // primary.povRight().onTrue(new MoveToPoint(m_elevator, 5, new Intake(m_coral, 0.7))); // L2
    // primary.povLeft().onTrue(new MoveToPoint(m_elevator, 17.5, new Intake(m_coral, 0.7))); // L3
    // primary.povUp().onTrue(new L4(m_elevator, m_coral)); // L4


    // CORAL - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    primary.leftTrigger().whileTrue(new Purge(m_coral, 0.5)); // Left trigger to purge coral
    primary.rightTrigger().whileTrue(new Intake(m_coral, 0.7)); // Right trigger to intake coral (auto-stop)

    // ALGAE - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    primary.y().whileTrue(new L3_Remove(m_elevator, m_algae));
    primary.b().whileTrue(new L2_Remove(m_elevator, m_algae));

    // Bound -
    // Right Trigger - Intake/Outtake
    // Right Bumper - Elevator Up
    // Left Trigger - Purge (Realign Coral)
    // Left Bumper - Elevator Down
    // Left Stick X - Drive Left/Right
    // Left Stick Y - Drive Forward/Backward
    // Right Stick X - Rotate
    // POV Up - L4
    // POV Down - L1
    // POV Left - L3
    // POV Right - L2
    // B - Remove Algae L2
    // Y - Remove Algae L3
    // A - Align to Reef

    // Unbound -
    // Left Stick Button
    // Right Stick Y
    // Right Stick Button
    // X
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
