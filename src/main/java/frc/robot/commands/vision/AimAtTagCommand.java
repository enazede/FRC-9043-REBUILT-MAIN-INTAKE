package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.Vision;

import java.util.function.DoubleSupplier;
import frc.robot.utils.Logger;

public class AimAtTagCommand extends Command {

	private final DrivetrainSubsystem drivetrain;
	private final Vision vision;
	private final PIDController aimController;
	private final DoubleSupplier manualRotationSupplier;
	private final DoubleSupplier xSpeedSupplier;
	private final DoubleSupplier ySpeedSupplier;

	private static final double kP = 0.55;
	private static final double kI = 0.0;
	private static final double kD = 0.02;
	private static final double YAW_DEADBAND_DEGREES = 5.0;
	private static final double MANUAL_ROTATION_DEADBAND = 0.08; // Stick merkezinde küçük oynama payı
	private static final double MAX_ROTATION_SPEED = 4.0; // rad/s civarı, gerektiğinde ayarlanabilir

	public AimAtTagCommand(
		DrivetrainSubsystem drivetrain,
    	Vision vision,
		DoubleSupplier manualRotationSupplier,
		DoubleSupplier xSpeedSupplier,
		DoubleSupplier ySpeedSupplier
		) {
		this.drivetrain = drivetrain;
		this.vision = vision;
		this.manualRotationSupplier = manualRotationSupplier;
		this.xSpeedSupplier = xSpeedSupplier;
		this.ySpeedSupplier = ySpeedSupplier;
		this.aimController = new PIDController(kP, kI, kD);
		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		aimController.reset();
	}

	@Override
	public void execute() {
		// Joystick değerlerine deadband uygula, küçük sapmalarda hız 0 olsun
		double rawManualRot = manualRotationSupplier.getAsDouble();
		
		double rawX = xSpeedSupplier.getAsDouble();
		double rawY = ySpeedSupplier.getAsDouble();

		double manualRot = MathUtil.applyDeadband(rawManualRot, MANUAL_ROTATION_DEADBAND);
		double xSpeed = MathUtil.applyDeadband(rawX, ControllerConstants.DEADBAND);
		double ySpeed = MathUtil.applyDeadband(rawY, ControllerConstants.DEADBAND);
		Translation2d translation = new Translation2d(xSpeed, ySpeed);

		// Hedef yoksa: sadece manuel sürüş ve manuel dönme
		if (vision == null || vision.getBestTarget() == null) {
			Logger.log("Vision/Aim/HasTarget", false);
			drivetrain.drive(translation, manualRot, true);
			return;
		}

		double yaw = vision.getBestTargetYaw();
		int targetId = vision.getBestTarget().getFiducialId();

		Pose3d[] SeenTargetPose = vision.getVisibleTagPoses();
		Pose3d BestTargetPose = vision.getBestTargetPose();

		Logger.log("Vision/Aim/HasTarget", true);
		Logger.log("Vision/Aim/0", 0);
		Logger.log("Vision/Aim/SeenTargetPose", SeenTargetPose);
		Logger.log("Vision/Aim/BestTargetPose", BestTargetPose);
		Logger.log("Vision/Aim/TargetId", targetId);
		Logger.log("Vision/Aim/YawDeg", yaw);

		if (Math.abs(yaw) < YAW_DEADBAND_DEGREES) {
			// Zaten yeterince hizalıysa: sadece manuel dönme (isterse hafif çevirebilsin)
			Logger.log("Vision/Aim/UsingPID", false);
			drivetrain.drive(translation, manualRot, true);
			return;
		}

		// Manuel override: stick ciddi şekilde oynuyorsa PID yerine elle kontrol et
		if (Math.abs(manualRot) > MANUAL_ROTATION_DEADBAND) {
			Logger.log("Vision/Aim/UsingPID", false);
			drivetrain.drive(translation, manualRot, true);
			return;
		}

		double rotationSpeed = aimController.calculate(yaw, 0.0);
		rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

		Logger.log("Vision/Aim/UsingPID", true);
		Logger.log("Vision/Aim/RotationCmd", rotationSpeed);

		drivetrain.drive(translation, rotationSpeed, true);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
