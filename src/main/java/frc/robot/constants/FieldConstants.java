package frc.robot.constants;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;

public class FieldConstants {
  private static final Distance BLUE_START_X = Meter.of(2.5);
  private static final Distance BLUE_START_Y = Meter.of(4);
  private static final Rotation2d BLUE_START_Degrees = Rotation2d.fromDegrees(0);

  private static final Distance RED_START_X = Meter.of(15.5);
  private static final Distance RED_START_Y = Meter.of(4);
  private static final Rotation2d RED_START_Degrees = Rotation2d.fromDegrees(180);

  public static final Pose2d BLUE_START_LOCATION = new Pose2d(new Translation2d(BLUE_START_X, BLUE_START_Y), BLUE_START_Degrees);
  public static final Pose2d RED_START_LOCATION = new Pose2d(new Translation2d(RED_START_X, RED_START_Y), RED_START_Degrees);

  private static final String layoutPath = Filesystem.getDeployDirectory() + "/apriltags/2026-rebuilt-welded.json";
  
  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = loadAprilTagLayout();

  private static AprilTagFieldLayout loadAprilTagLayout() {
    try {
      return new AprilTagFieldLayout(layoutPath);
    } catch (IOException e) {
      System.err.println("2026 AprilTag Cant Load! Href: " + layoutPath);
      e.printStackTrace();
      return null; 
    }
  }
}