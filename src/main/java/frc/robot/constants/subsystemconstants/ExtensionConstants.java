package frc.robot.constants.subsystemconstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ExtensionConstants {
  // Real Constants
  public static final int setIntakeMotor1ID = 7;
  public static final MotorType setIntakeMotor1type = MotorType.kBrushless;

  public static final SparkBaseConfig setIntakeMotor1Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kBrake)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);

  // Real Constants
  public static final int setIntakeMotor2ID = 14;
  public static final MotorType setIntakeMotor2type = MotorType.kBrushless;

  public static final SparkBaseConfig setIntakeMotor2Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kBrake)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);
  }
