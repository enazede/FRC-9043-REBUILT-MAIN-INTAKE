package frc.robot.subsystems.extension;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class ExtensionIOSim implements ExtensionIO {
  public ExtensionIOSim(SwerveDriveSimulation driveSim) {}

  @Override
  public void extensionOutPut(double percentSpeed){} 

  public void end(){}
}