package frc.robot.subsystems.vision;

import java.util.List;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.subsystemconstants.VisionConstants;
import frc.robot.utils.Logger;

public class VisionIOReal implements VisionIO {

    private final PhotonCamera cameraLeft;
    private final PhotonCamera cameraRight;
    
    private PhotonPipelineResult result;
    private PhotonPipelineResult leftResult;
    private PhotonPipelineResult rightResult;
    private Transform3d activeRobotToCameraTransform;

    public VisionIOReal(String cameraname,String cameraname2) {
        this.cameraLeft = new PhotonCamera(cameraname);
        this.cameraRight = new PhotonCamera(cameraname2);
        this.result = new PhotonPipelineResult();
        this.leftResult = new PhotonPipelineResult();
        this.rightResult = new PhotonPipelineResult();
        this.activeRobotToCameraTransform = VisionConstants.ROBOT_TO_CAMERA_LEFT;
    }

    @Override
    public void update() {
        List<PhotonPipelineResult> leftUnreadResults = cameraLeft.getAllUnreadResults();
        List<PhotonPipelineResult> rightUnreadResults = cameraRight.getAllUnreadResults();

        if (!leftUnreadResults.isEmpty()) {
            this.leftResult = leftUnreadResults.get(leftUnreadResults.size() - 1);
        }
        if (!rightUnreadResults.isEmpty()) {
            this.rightResult = rightUnreadResults.get(rightUnreadResults.size() - 1);
        }

        this.result = pickBetterResult(leftResult, rightResult);
        logVisionSelection();
    }

    @Override
    public PhotonTrackedTarget getBestTarget() {
       PhotonTrackedTarget leftBest = leftResult.hasTargets() ? leftResult.getBestTarget() : null;
       PhotonTrackedTarget rightBest = rightResult.hasTargets() ? rightResult.getBestTarget() : null;
       return pickBetterTarget(leftBest, rightBest);
    }

    @Override
    public PhotonTrackedTarget getTarget(AprilTagID aprilTagID) {
        PhotonTrackedTarget leftTarget = getTargetFromResult(leftResult, aprilTagID);
        PhotonTrackedTarget rightTarget = getTargetFromResult(rightResult, aprilTagID);
        return pickBetterTarget(leftTarget, rightTarget);
    }

    @Override
    public boolean isSeen(AprilTagID aprilTagID) {
        return getTarget(aprilTagID) != null;
    }

    @Override
    public List<PhotonTrackedTarget> getSeenTargets() {
        List<PhotonTrackedTarget> seenTargets = new ArrayList<>();
        if (leftResult.hasTargets()) {
            seenTargets.addAll(leftResult.getTargets());
        }
        if (rightResult.hasTargets()) {
            seenTargets.addAll(rightResult.getTargets());
        }
        return seenTargets;
    }

    @Override
    public PhotonPipelineResult getPipelineResult() {
        return result;
    }

    @Override
    public Transform3d getRobotToCameraTransform() {
        return activeRobotToCameraTransform;
    }

    private PhotonTrackedTarget getTargetFromResult(PhotonPipelineResult pipelineResult, AprilTagID aprilTagID) {
        if (!pipelineResult.hasTargets()) {
            return null;
        }
        for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
            if (target.getFiducialId() == aprilTagID.id) {
                return target;
            }
        }
        return null;
    }

    private PhotonPipelineResult pickBetterResult(PhotonPipelineResult left, PhotonPipelineResult right) {
        PhotonTrackedTarget leftBest = left.hasTargets() ? left.getBestTarget() : null;
        PhotonTrackedTarget rightBest = right.hasTargets() ? right.getBestTarget() : null;

        PhotonTrackedTarget bestTarget = pickBetterTarget(leftBest, rightBest);
        if (bestTarget == null) {
            activeRobotToCameraTransform = VisionConstants.ROBOT_TO_CAMERA_LEFT;
            return new PhotonPipelineResult();
        }
        if (bestTarget == leftBest) {
            activeRobotToCameraTransform = VisionConstants.ROBOT_TO_CAMERA_LEFT;
            return left;
        }
        activeRobotToCameraTransform = VisionConstants.ROBOT_TO_CAMERA_RIGHT;
        return right;
    }

    private PhotonTrackedTarget pickBetterTarget(PhotonTrackedTarget first, PhotonTrackedTarget second) {
        if (first == null) return second;
        if (second == null) return first;

        // Önce daha düşük ambiguity (daha güvenilir pose), sonra daha yüksek alan (daha net görüntü)
        double firstAmbiguity = normalizedAmbiguity(first.getPoseAmbiguity());
        double secondAmbiguity = normalizedAmbiguity(second.getPoseAmbiguity());
        if (firstAmbiguity < secondAmbiguity) return first;
        if (secondAmbiguity < firstAmbiguity) return second;

        double firstArea = first.getArea();
        double secondArea = second.getArea();
        if (firstArea > secondArea) return first;
        if (secondArea > firstArea) return second;

        // Son care merkeze daha yakın yaw
        return Math.abs(first.getYaw()) <= Math.abs(second.getYaw()) ? first : second;
    }

    private double normalizedAmbiguity(double ambiguity) {
        return ambiguity < 0.0 ? Double.POSITIVE_INFINITY : ambiguity;
    }

    private void logVisionSelection() {
        PhotonTrackedTarget leftBest = leftResult.hasTargets() ? leftResult.getBestTarget() : null;
        PhotonTrackedTarget rightBest = rightResult.hasTargets() ? rightResult.getBestTarget() : null;
        PhotonTrackedTarget selectedBest = result.hasTargets() ? result.getBestTarget() : null;

        Logger.log("Vision/Left/HasTarget", leftBest != null);
        Logger.log("Vision/Right/HasTarget", rightBest != null);
        Logger.log("Vision/Selected/HasTarget", selectedBest != null);
        Logger.log("Vision/Selected/Camera", activeRobotToCameraTransform == VisionConstants.ROBOT_TO_CAMERA_LEFT ? "left" : "right");

        if (leftBest != null) {
            Logger.log("Vision/Left/TargetId", leftBest.getFiducialId());
            Logger.log("Vision/Left/YawDeg", leftBest.getYaw());
            Logger.log("Vision/Left/Ambiguity", leftBest.getPoseAmbiguity());
            Logger.log("Vision/Left/Area", leftBest.getArea());
        }

        if (rightBest != null) {
            Logger.log("Vision/Right/TargetId", rightBest.getFiducialId());
            Logger.log("Vision/Right/YawDeg", rightBest.getYaw());
            Logger.log("Vision/Right/Ambiguity", rightBest.getPoseAmbiguity());
            Logger.log("Vision/Right/Area", rightBest.getArea());
        }

        if (selectedBest != null) {
            Logger.log("Vision/Selected/TargetId", selectedBest.getFiducialId());
            Logger.log("Vision/Selected/YawDeg", selectedBest.getYaw());
            Logger.log("Vision/Selected/Ambiguity", selectedBest.getPoseAmbiguity());
            Logger.log("Vision/Selected/Area", selectedBest.getArea());
            Logger.log("Vision/Selected/DistanceM", selectedBest.getBestCameraToTarget().getTranslation().getNorm());
        }
    }
}