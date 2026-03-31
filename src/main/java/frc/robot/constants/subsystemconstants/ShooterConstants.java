package frc.robot.constants.subsystemconstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConstants {
  // Simulation Constants

  public static final Translation2d FUEL_SHOOTER_POSITION = new Translation2d(0.22, 0.1); // Shooter'ın robot üzerindeki ofseti (metre)
  public final static Distance FUEL_SHOOTER_HEIGHT = Meters.of(0.3);  // Fırlatılış yüksekliği
  public static final LinearVelocity FUEL_SHOOTER_SPEED = MetersPerSecond.of(7.5); // Fırlatılış hızı m/s
  public static final Angle FUEL_SHOOTER_ANGLE = Degrees.of(70); // Fırlatılış açısı derece
  public static final double FUEL_SHOOTER_TIME = 0.25; // Saniyede 4 top = her 0.25 saniyede bir atış (1 / 4 = 0.25)
  
  public class FuelShooterConstants2 {
    public static final Translation2d FUEL_SHOOTER_POSITION = new Translation2d(0.22, -0.1); // Shooter'ın robot üzerindeki ofseti (metre)
    public final static Distance FUEL_SHOOTER_HEIGHT = Meters.of(0.3);  // Fırlatılış yüksekliği
        
    public static final LinearVelocity FUEL_SHOOTER_SPEED = MetersPerSecond.of(7.5); // Fırlatılış hızı m/s

    public static final Angle FUEL_SHOOTER_ANGLE = Degrees.of(60); // Fırlatılış açısı derece
    
    public static final double FUEL_SHOOTER_TIME = 0.25; // Saniyede 4 top = her 0.25 saniyede bir atış (1 / 4 = 0.25)
  }

  // Real Constants

  public static final int indexerMotor1ID = 4;
  public static final MotorType indexerMotor1type = MotorType.kBrushless;  
  
  public static final SparkBaseConfig indexerMotor1Config = new SparkMaxConfig()
  .smartCurrentLimit(60)
  .idleMode(IdleMode.kCoast)
  .inverted(false)
  .openLoopRampRate(0.2)
  .closedLoopRampRate(0.05)
  .voltageCompensation(12);

  public static final int shooterMotor1ID = 1;
  
  public static final TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
    static{
      shooterMotor1Config.Slot0.kP = 0.12;
      shooterMotor1Config.Slot0.kV = 0.13;

      shooterMotor1Config.CurrentLimits.StatorCurrentLimitEnable = true;
      shooterMotor1Config.CurrentLimits.StatorCurrentLimit = 65;

      shooterMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      shooterMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;
      shooterMotor1Config.CurrentLimits.SupplyCurrentLimit = 40;

      shooterMotor1Config.Voltage.PeakForwardVoltage = 12;
      shooterMotor1Config.Voltage.PeakReverseVoltage = -12;
    }

  /**
   * Photon {@code cameraToTarget} çeviri normu (m). Yakın mesafede düşük RPS, uzakta yüksek RPS.
   * Saha testine göre {@link #VISION_SHOOT_DISTANCE_NEAR_M} / {@link #VISION_SHOOT_DISTANCE_FAR_M}
   * ve RPS uçlarını kalibre edin.
   */
  public static final double VISION_SHOOT_DISTANCE_NEAR_M = 1.2;
  public static final double VISION_SHOOT_DISTANCE_FAR_M = 4.8;
  public static final double VISION_SHOOT_RPS_NEAR = 50.0;
  public static final double VISION_SHOOT_RPS_FAR = 83.0;

  public static double targetRpsForAprilTagDistanceMeters(double distanceM) {
    double span = VISION_SHOOT_DISTANCE_FAR_M - VISION_SHOOT_DISTANCE_NEAR_M;
    if (span <= 0) {
      // Uzak ≤ yakın ise interpolasyon tanımsız; sabitleri düzelt
      return (VISION_SHOOT_RPS_NEAR + VISION_SHOOT_RPS_FAR) / 2.0;
    }
    double t = (distanceM - VISION_SHOOT_DISTANCE_NEAR_M) / span;
    t = MathUtil.clamp(t, 0.0, 1.0);
    return VISION_SHOOT_RPS_NEAR + t * (VISION_SHOOT_RPS_FAR - VISION_SHOOT_RPS_NEAR);
  }
}
