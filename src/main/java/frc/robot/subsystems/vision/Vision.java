package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FieldConstants;
import frc.robot.constants.subsystemconstants.VisionConstants;

public class Vision extends SubsystemBase {

  VisionIO camera;

  PhotonPoseEstimator estimator;

  public Vision(VisionIO camera) {
    this.camera = camera;

    this.estimator = new PhotonPoseEstimator(
      FieldConstants.APRILTAG_FIELD_LAYOUT, 
      VisionConstants.ESTIMATION_STRATEGY, 
      VisionConstants.ROBOT_TO_CAMERA
    );
  }

  @Override
  public void periodic() {
    camera.update();
  }

  public double getBestTargetYaw() {
    if (getBestTarget() != null) {
      return getBestTarget().getYaw();
    }
    return 0.0;
  }

  public boolean canEstimatePose() {
    estimator.setRobotToCameraTransform(camera.getRobotToCameraTransform());
    return estimator.update(camera.getPipelineResult()).isPresent();
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    estimator.setRobotToCameraTransform(camera.getRobotToCameraTransform());
    return estimator.update(camera.getPipelineResult());
  }

  public boolean isSeen(AprilTagID id) {
    return camera.isSeen(id);
  }

  public PhotonTrackedTarget getTarget(AprilTagID id) {
    return camera.getTarget(id);
  }
  
  /**
   * Seçilen en iyi hedef için kamera–tag dönüşümü (Photon 3B çözümü).
   * {@link #getBestTarget()} ile aynı hedeften okunur; çift kamerada pipeline
   * {@code getBestTarget()} ile çelişmeyecek şekilde mesafe tek kaynakta toplanır.
   */
  public Transform3d getTargetTransform() {
    PhotonTrackedTarget best = getBestTarget();
    if (best == null) {
      return null;
    }
    return pickUsableCameraToTarget(best);
  }

  /** best/alternate pose çözümlerinden kullanılabilir (sıfır olmayan) mesafe veren dönüşüm. */
  private static Transform3d pickUsableCameraToTarget(PhotonTrackedTarget t) {
    Transform3d primary = t.getBestCameraToTarget();
    if (isUsableDistanceMeters(primary.getTranslation().getNorm())) {
      return primary;
    }
    Transform3d alternate = t.getAlternateCameraToTarget();
    if (isUsableDistanceMeters(alternate.getTranslation().getNorm())) {
      return alternate;
    }
    return primary;
  }

  private static boolean isUsableDistanceMeters(double distanceM) {
    return distanceM > 1e-3 && !Double.isNaN(distanceM) && !Double.isInfinite(distanceM);
  }

  /** En iyi hedef için kamera–tag mesafesi (m), yalnızca Photon hedef verisi; yoksa boş. */
  public OptionalDouble getBestTargetDistanceMeters() {
    PhotonTrackedTarget best = getBestTarget();
    if (best == null) {
      return OptionalDouble.empty();
    }
    double d = pickUsableCameraToTarget(best).getTranslation().getNorm();
    if (!isUsableDistanceMeters(d)) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(d);
  }

  public PhotonTrackedTarget getBestTarget() {
    return camera.getBestTarget();
  }

  public Pose3d[] getVisibleTagPoses() {
    return camera.getPipelineResult().getTargets().stream()
      .map(target -> FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(target.getFiducialId()))
      .flatMap(java.util.Optional::stream)
      .toArray(Pose3d[]::new);
  }
  public Pose3d getBestTargetPose() {
    var bestTarget = getBestTarget();
    if (bestTarget != null) {
      // ID'yi alıp sahadaki sabit koordinatını layout'tan çekiyoruz
      return FieldConstants.APRILTAG_FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId()).orElse(null);
    }
    return null;
  }
  
}
