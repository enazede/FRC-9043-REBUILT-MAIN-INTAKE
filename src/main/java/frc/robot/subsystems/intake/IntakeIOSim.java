package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.constants.subsystemconstants.IntakeConstants;
import frc.robot.utils.Logger;

public class IntakeIOSim implements IntakeIO {
  private final IntakeSimulation intakeSim;

  public IntakeIOSim(SwerveDriveSimulation driveSim) {
    this.intakeSim = IntakeSimulation.OverTheBumperIntake(
      IntakeConstants.GAME_PIECE,
      driveSim,
      IntakeConstants.FUEL_INTAKE_WIDTH,
      IntakeConstants.FUEL_INTAKE_LENGTH, 
      IntakeConstants.FUEL_INTAKE_SIDE,
      IntakeConstants.FUEL_INTAKE_CAPATICY 
    );
  }
  
  @Override
  public void intakeOutput(double percentSpeed) {
    intakeSim.startIntake();
    Logger.log("Intake/FuelCount", getGamePieceCount());
  }

  private int getGamePieceCount() {
    return intakeSim.getGamePiecesAmount();
  }

  public void end(){
    intakeSim.stopIntake();
  }
}