package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class AutoConstants {
  
  public static final double DRIVE_P = 5;
  public static final double DRIVE_I = 0.0;
  public static final double DRIVE_D = 0.0;
  public static final double DRIVE_IZ = 0.0;

  public static final PIDConstants DRIVE_PID = new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_IZ);

  public static final Constraints DRIVE_CONSTRAINTS = new Constraints(
    MetersPerSecond.of(1000).in(MetersPerSecond), 
    MetersPerSecondPerSecond.of(1000).in(MetersPerSecondPerSecond)
  );

  public static final Distance DISTANCE_TOLERANCE = Meters.of(0.1);
  public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.05);
  
  public static final double ANGLE_P = 5;
  public static final double ANGLE_I = 0.0;
  public static final double ANGLE_D = 0.0;
  public static final double ANGLE_IZ = 0.0;

  public static final PIDConstants ANGLE_PID = new PIDConstants(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_IZ);
  
  public static final Constraints ANGLE_CONSTRAINTS = new Constraints(
    DegreesPerSecond.of(1000).in(DegreesPerSecond), 
    DegreesPerSecondPerSecond.of(1000).in(DegreesPerSecondPerSecond)
  );

  public static final Angle ANGLE_TOLERANCE = Degrees.of(5);
  public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE = DegreesPerSecond.of(2);

}
