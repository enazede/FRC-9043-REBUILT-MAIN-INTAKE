package frc.robot.constants.subsystemconstants;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConstants {
  // Simulation Constants
  public static final String GAME_PIECE = "Fuel";
  public static final int FUEL_INTAKE_CAPATICY = 40; // (max100)
  public static final IntakeSide FUEL_INTAKE_SIDE = IntakeSimulation.IntakeSide.BACK; 
  public static final Distance FUEL_INTAKE_WIDTH = Meters.of(0.3); 
  public static final Distance FUEL_INTAKE_LENGTH = Meters.of(0.2);

  // Real Constants
  public static final int intakeMotor1ID = 5;
  public static final MotorType intakeMotor1type = MotorType.kBrushless;

  public static final SparkBaseConfig intakeMotor1Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kCoast)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);
  
  // Real Constants
  public static final int intakeMotor2ID = 10;
  public static final MotorType intakeMotor2type = MotorType.kBrushless;

  public static final SparkBaseConfig intakeMotor2Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kCoast)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);
  }
