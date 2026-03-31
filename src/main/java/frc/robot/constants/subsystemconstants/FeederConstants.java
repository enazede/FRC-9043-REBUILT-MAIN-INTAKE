package frc.robot.constants.subsystemconstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FeederConstants {
  // Real Constants
  public static final int feederMotor1ID = 9;
  public static final MotorType feederMotor1type = MotorType.kBrushless;

  public static final SparkBaseConfig feederMotor1Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kCoast)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);
}
