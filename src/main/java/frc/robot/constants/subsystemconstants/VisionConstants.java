package frc.robot.constants.subsystemconstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

  public static final PoseStrategy ESTIMATION_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final Transform3d ROBOT_TO_CAMERA_LEFT = new Transform3d(
    Meters.of(-0.25), // x (roboRIO'dan 25 cm geride)
    Meters.of(0.20), // y (sola 20 cm)
    Meters.of(0.50), // z (yukari 50 cm)
    new Rotation3d(
      Degrees.of(0),      
      Degrees.of(-15),      
      Degrees.of(0)      
    )
  );
  public static final Transform3d ROBOT_TO_CAMERA_RIGHT = new Transform3d(
    Meters.of(-0.25), // x (roboRIO'dan 25 cm geride)
    Meters.of(-0.20), // y (saga 20 cm)
    Meters.of(0.50), // z (yukari 50 cm)
    new Rotation3d(
      Degrees.of(0),
      Degrees.of(-15),
      Degrees.of(0)
    )
  );
  // Geriye uyumluluk: tek kamera/sim kullanımlarında sol kamerayı varsayılan tut.
  public static final Transform3d ROBOT_TO_CAMERA = ROBOT_TO_CAMERA_LEFT;
  
  public static SimCameraProperties CAMERA_PROPERTIES = new SimCameraProperties();

  public static final String CAMERA_NAME = "erdem_left";
  public static final String CAMERA_NAME2 = "kayra_right";

  public static final String VISION_SYSTEM_NAME = "Main";

  static {
    CAMERA_PROPERTIES.setCalibration(640, 480, Rotation2d.fromDegrees(90));
    CAMERA_PROPERTIES.setCalibError(0.25, 0.08);
    CAMERA_PROPERTIES.setFPS(30);
    CAMERA_PROPERTIES.setAvgLatencyMs(50);
    CAMERA_PROPERTIES.setLatencyStdDevMs(10);
  }
}
