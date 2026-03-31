package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.Logger;
import frc.robot.constants.MotorConstants;
import frc.robot.constants.subsystemconstants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

  private final SparkMax indexerMotor1;
  private final TalonFX shooterMotor1;

  public static double targetRPS = 55.0;
  //public static double targetRPM = 500.0;
  public static double targetPercentSpeed = 0.7;

  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public ShooterIOReal() {
    this.indexerMotor1 = new SparkMax(ShooterConstants.indexerMotor1ID, ShooterConstants.indexerMotor1type);    
    this.shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID);
    setConfigs();
  }

  private void setConfigs(){
    this.shooterMotor1.getConfigurator().apply(ShooterConstants.shooterMotor1Config);
    this.indexerMotor1.configure(ShooterConstants.indexerMotor1Config, MotorConstants.resetMode, MotorConstants.persistMode);
  }

  @Override
  public void indexerOutput(double percentSpeed) {
    indexerMotor1.set(percentSpeed);
  }

  @Override
  public void shoot(boolean run){
    if(run){
      shootOutput(targetRPS);
      double currentRPS = shooterMotor1.getVelocity().getValueAsDouble();
      Logger.log("FieldSimulation/ShootCurrentRPS",currentRPS);
      double absCur = Math.abs(currentRPS);
      double absTgt = Math.abs(targetRPS);
      if (absCur * 0.98 >= absTgt) {
        indexerOutput(targetPercentSpeed);
      }
      else{
        indexerOutput(0);
      }
    }
    else{
      end();
    }
  }

  @Override
  public void shootOutput(double rps) {
    shooterMotor1.setControl(velocityControl.withVelocity(rps));
  }

  public void increaseSpeed() {
    targetRPS += 1.0;
    if (targetRPS > 100.0){
      targetRPS = 100.0;
    }
    Logger.log("FieldSimulation/ShootTargetRPS",targetRPS);
  }

  public void decreaseSpeed() {
    targetRPS -= 1.0;
    if (targetRPS < 0){
      targetRPS = 0;
    }
    Logger.log("FieldSimulation/ShootTargetRPS",targetRPS);
  }

  @Override
  public void setTargetRps(double rps) {
    targetRPS = MathUtil.clamp(rps, 0.0, 100.0);
    Logger.log("FieldSimulation/ShootTargetRPS", targetRPS);
  }

  public void end() {
    indexerMotor1.set(0);
    shooterMotor1.stopMotor();
  }
}