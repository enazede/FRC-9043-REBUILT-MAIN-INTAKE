package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.MotorConstants;
import frc.robot.constants.subsystemconstants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

  private final SparkMax intakeMotor1, intakeMotor2;

  public IntakeIOReal() {
    this.intakeMotor1 = new SparkMax(IntakeConstants.intakeMotor1ID, IntakeConstants.intakeMotor1type);
    this.intakeMotor2 = new SparkMax(IntakeConstants.intakeMotor2ID, IntakeConstants.intakeMotor2type);
    setConfigs();
  }

  private void setConfigs(){
    this.intakeMotor1.configure(IntakeConstants.intakeMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
    this.intakeMotor2.configure(IntakeConstants.intakeMotor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  @Override
  public void intakeOutput(double percentSpeed) {
    intakeMotor1.set(percentSpeed);
    intakeMotor2.set(percentSpeed);
  }

  public void end(){
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }
}