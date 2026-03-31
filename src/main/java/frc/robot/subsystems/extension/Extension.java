package frc.robot.subsystems.extension;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extension extends SubsystemBase {
  private final ExtensionIO io;

  public Extension(ExtensionIO io) {
    this.io = io;
  }

  public void extensionOutPut(double percentSpeed) {
    io.extensionOutPut(percentSpeed);
  }

  public void end(){
    io.end();
  };
}