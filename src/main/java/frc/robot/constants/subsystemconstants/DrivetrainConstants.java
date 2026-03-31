package frc.robot.constants.subsystemconstants;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class DrivetrainConstants {

  public static final double P = 0.1;
  public static final double I = 0.001;
  public static final double D = 0.1;
  public static final double IZ = 0.2;

  public static final double S = 0.0;
  public static final double G = 0.3;
  public static final double V = 0.0;
  public static final double A = 0.0;

  private static final Distance BUMPER_SIZE_X = Meters.of(0.8);
  private static final Distance BUMPER_SIZE_Y = Meters.of(0.8);

  private static final Mass ROBOT_MASS = Kilograms.of(57.924);

  private static final double WHEELCOF = 1.19;
  private static final int GEAR_RATIO_LEVEL = 6;

  public static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = DriveTrainSimulationConfig.Default()
  .withBumperSize(BUMPER_SIZE_X, BUMPER_SIZE_Y)
  .withRobotMass(ROBOT_MASS)
  .withGyro(COTS.ofNav2X())
  .withSwerveModule(
    COTS.ofThriftySwerve(
      DCMotor.getNEO(1), 
      DCMotor.getNEO(1), 
      WHEELCOF, 
      GEAR_RATIO_LEVEL)
    );

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(Meters.of(+0.3310000009), Meters.of(+0.2679999999)),
    new Translation2d(Meters.of(+0.3310000009), Meters.of(-0.2679999999)),
    new Translation2d(Meters.of(-0.3310000009), Meters.of(+0.2679999999)),
    new Translation2d(Meters.of(-0.3310000009), Meters.of(-0.2679999999))
  ); 
}
