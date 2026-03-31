package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void intakeOutput(double percentSpeed) {
    io.intakeOutput(percentSpeed);
  }

  @Override
  public void periodic() {
  }
  
  public void end(){
    io.end();
  };
}