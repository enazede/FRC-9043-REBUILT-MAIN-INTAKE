package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.RobotMode;
import swervelib.math.Matter;

import com.pathplanner.lib.config.RobotConfig;

public class RobotConstants {
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(-10, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static RobotMode ROBOT_MODE = RobotMode.Normal;
  public static RobotConfig ROBOT_CONFIG;
  static {
    try {
      ROBOT_CONFIG = RobotConfig.fromGUISettings();
    } catch (Exception e){
      e.printStackTrace();
    }
  }
  
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }
}
