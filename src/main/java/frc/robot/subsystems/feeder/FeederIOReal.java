package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;

import frc.robot.constants.MotorConstants;
import frc.robot.constants.subsystemconstants.FeederConstants;

public class FeederIOReal implements FeederIO {
  
  private final SparkMax feederMotor1;

  public FeederIOReal() {
    this.feederMotor1 = new SparkMax(FeederConstants.feederMotor1ID, FeederConstants.feederMotor1type);
    setConfigs();
  }

  private void setConfigs(){
    this.feederMotor1.configure(FeederConstants.feederMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  @Override
  public void setOutput(double speed) {
    feederMotor1.set(speed);
  }

  @Override
  public void end(){
    feederMotor1.set(0);
  }
}