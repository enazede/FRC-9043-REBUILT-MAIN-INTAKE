package frc.robot.subsystems.extension;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.MotorConstants;
import frc.robot.constants.subsystemconstants.ExtensionConstants;

public class ExtensionIOReal implements ExtensionIO {

  private final SparkMax setIntakeMotor1, setIntakeMotor2;

  public ExtensionIOReal() {
    this.setIntakeMotor1 = new SparkMax(ExtensionConstants.setIntakeMotor1ID, ExtensionConstants.setIntakeMotor1type); 
    this.setIntakeMotor2 = new SparkMax(ExtensionConstants.setIntakeMotor2ID, ExtensionConstants.setIntakeMotor2type); 
    setConfigs();
  }

  private void setConfigs(){
    this.setIntakeMotor1.configure(ExtensionConstants.setIntakeMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
    this.setIntakeMotor2.configure(ExtensionConstants.setIntakeMotor2Config, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  @Override
  public void extensionOutPut(double percentSpeed){
    setIntakeMotor1.set(percentSpeed);
    setIntakeMotor2.set(-percentSpeed);
  }

  @Override
  public void end(){
    setIntakeMotor1.set(0);
    setIntakeMotor2.set(0);
  }

}