package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.utils.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final SparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder;

    public Algae() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false); // Change in case of wrong direction
        this.intakeMotor = new SparkMax(AlgaeConstants.algaeIntakeCANId, MotorType.kBrushless);
        this.intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.pivotMotor = new SparkMax(AlgaeConstants.algaePivotCANId, MotorType.kBrushless);
        this.pivotEncoder = pivotMotor.getAbsoluteEncoder();
    }

    public void intakeAlgae(double speed) {
        this.intakeMotor.set(speed);
    }

    public void ejectAlgae(double speed) {
        this.intakeMotor.set(speed);
    }

    public void pivotAlgae(double speed) {
        this.pivotMotor.set(speed);
    }

    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }

    public void stopPivot() {
        this.pivotMotor.stopMotor();
    }

    public double getPivotPosition() {
        return this.pivotEncoder.getPosition();
    }

}