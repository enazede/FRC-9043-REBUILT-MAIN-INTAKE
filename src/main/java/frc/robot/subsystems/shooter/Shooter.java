package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  private float score = 0;
  private float shootCount = 0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic(){
    Logger.log("Shooter/Succsesful Score",score);
    if(score > 0) {
      Logger.log("Shooter/xG",score/shootCount);
    }
  }

  public void indexerOutput(double percentSpeed) {
    io.indexerOutput(percentSpeed);
  }

  public void shootOutput(double rps){
    io.shootOutput(rps);
  }

  public void increaseSpeed() {
    io.increaseSpeed();
  }

  public void decreaseSpeed() {
    io.decreaseSpeed();
  }

  public void setTargetRps(double rps) {
    io.setTargetRps(rps);
  }

  public void shoot(boolean run){
    io.shoot(run);
  }
}