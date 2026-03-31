package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public void end(){
    io.end();
  };
}