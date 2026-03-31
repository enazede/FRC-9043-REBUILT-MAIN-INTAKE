package frc.robot.subsystems.shooter;

public interface ShooterIO {
  void indexerOutput(double percentSpeed);

  void shootOutput(double rps);
  
  void increaseSpeed();

  void decreaseSpeed();

  void shoot(boolean run);

  default void setTargetRps(double rps) {}
}
