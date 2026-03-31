// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.TeleopConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.math.SwerveMath;

/**
 * Teleop sürüş: saha göre hızlar {@link DrivetrainSubsystem#driveFieldOriented(ChassisSpeeds)} ile verilir
 * (9043 REEFSCAPE ile aynı mantık: mavi tüm eksenler, kırmızıda x/y/ω işaret çevirmesi).
 */
public class FieldRelativeDriveCommand extends Command {

  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier angularRotationX;

  public FieldRelativeDriveCommand(
      DrivetrainSubsystem drivetrain,
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    this.drivetrain = drivetrain;
    this.translationX = translationX;
    this.translationY = translationY;
    this.angularRotationX = angularRotationX;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double tx =
        MathUtil.applyDeadband(translationX.getAsDouble(), ControllerConstants.DEADBAND);
    double ty =
        MathUtil.applyDeadband(translationY.getAsDouble(), ControllerConstants.DEADBAND);
    double rotRaw =
        MathUtil.applyDeadband(angularRotationX.getAsDouble(), ControllerConstants.DEADBAND);

    var swerve = drivetrain.getSwerveDrive();
    Translation2d scaled =
        SwerveMath.scaleTranslation(
            new Translation2d(
                tx * swerve.getMaximumChassisVelocity(), ty * swerve.getMaximumChassisVelocity()),
            TeleopConstants.TRANSLATION_SCALE);
    double omega =
        Math.pow(rotRaw, TeleopConstants.ROTATION_EXPONENT)
            * swerve.getMaximumChassisAngularVelocity();

    boolean isBlue =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
    double vx = scaled.getX();
    double vy = scaled.getY();
    if (!isBlue) {
      vx = -vx;
      vy = -vy;
      omega = -omega;
    }

    drivetrain.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
