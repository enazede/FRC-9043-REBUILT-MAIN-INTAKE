package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;

import frc.robot.constants.subsystemconstants.ShooterConstants;
import frc.robot.utils.Logger;

public class ShooterIOSim implements ShooterIO {
  private final SwerveDriveSimulation driveSim;
  private double lastShotTime = 0;
  private final double COOLDOWN_TIME = 0.5;

  /** ShooterIOReal ile aynı varsayılan; sim fişek hızı buna göre ölçeklenir. */
  private double targetRPS = 60.0;
  private static final double BASELINE_RPS = 60.0;
  private static final double LAUNCH_SCALE_MIN = 0.2;
  private static final double LAUNCH_SCALE_MAX = 1.4;

  public ShooterIOSim(SwerveDriveSimulation driveSim) {
    this.driveSim = driveSim;
  }

  private LinearVelocity launchSpeedForCurrentRps() {
    double baseMps = ShooterConstants.FUEL_SHOOTER_SPEED.in(MetersPerSecond);
    // BASELINE_RPS sabit 60 — paydada 0 yok; targetRPS / 60 = 7.5 m/s tabanına göre oran
    double scale = targetRPS / BASELINE_RPS;
    scale = MathUtil.clamp(scale, LAUNCH_SCALE_MIN, LAUNCH_SCALE_MAX);
    return MetersPerSecond.of(baseMps * scale);
  }

  @Override
  public void shoot(boolean run) {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (run && (now - lastShotTime >= COOLDOWN_TIME)) {
      lastShotTime = now;
      LinearVelocity launchSpeed = launchSpeedForCurrentRps();
      RebuiltFuelOnFly fuelProjectile = new RebuiltFuelOnFly(
        driveSim.getSimulatedDriveTrainPose().getTranslation(),
        ShooterConstants.FUEL_SHOOTER_POSITION,
        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        driveSim.getSimulatedDriveTrainPose().getRotation(),
        ShooterConstants.FUEL_SHOOTER_HEIGHT,
        launchSpeed,
        ShooterConstants.FUEL_SHOOTER_ANGLE
      );
      RebuiltFuelOnFly fuelProjectile2 = new RebuiltFuelOnFly(
        driveSim.getSimulatedDriveTrainPose().getTranslation(),
        ShooterConstants.FuelShooterConstants2.FUEL_SHOOTER_POSITION,
        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        driveSim.getSimulatedDriveTrainPose().getRotation(),
        ShooterConstants.FuelShooterConstants2.FUEL_SHOOTER_HEIGHT,
        launchSpeed,
        ShooterConstants.FUEL_SHOOTER_ANGLE
      );
      SimulatedArena.getInstance().addGamePieceProjectile(fuelProjectile);
      SimulatedArena.getInstance().addGamePieceProjectile(fuelProjectile2);
    }
  }

  @Override
  public void shootOutput(double rps) {
    setTargetRps(rps);
  }

  @Override
  public void indexerOutput(double percentSpeed) {}

  @Override
  public void increaseSpeed() {
    setTargetRps(targetRPS + 1.0);
  }

  @Override
  public void decreaseSpeed() {
    setTargetRps(targetRPS - 1.0);
  }

  @Override
  public void setTargetRps(double rps) {
    targetRPS = MathUtil.clamp(rps, 0.0, 100.0);
    Logger.log("FieldSimulation/ShootTargetRPS", targetRPS);
  }
}
