// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.drivetrain.FieldRelativeDriveCommand;
import frc.robot.commands.vision.AimAtTagCommand;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.subsystemconstants.ShooterConstants;
import frc.robot.constants.subsystemconstants.VisionConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.extension.ExtensionIOReal;
import frc.robot.subsystems.extension.ExtensionIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Logger;

import java.io.File;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.simulation.VisionSystemSim;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  
  /* <--------------------------------------------------------------------------------------------------------------------> */
  
  private final CommandPS5Controller driverController = new CommandPS5Controller(ControllerConstants.CONTROLLER_PORT);

  private final Trigger square = driverController.square();
  private final Trigger cross = driverController.cross();
  private final Trigger circle = driverController.circle();
  private final Trigger ps = driverController.PS();
  private final Trigger triangle = driverController.triangle();
  private final Trigger r1 = driverController.R1();
  // private final Trigger r2 = driverController.R2();
  private final Trigger r3 = driverController.R3();
  private final Trigger l1 = driverController.L1();
  private final Trigger l2 = driverController.L2();
  private final Trigger l3 = driverController.L3();
  private final Trigger options = driverController.options();

  /* <--------------------------------------------------------------------------------------------------------------------> */

  private DrivetrainSubsystem drivetrain;
  private Intake intake;
  private Shooter shooter;
  private Vision vision;
  private VisionSystemSim visionSim;
  private Extension extension;
  private Feeder feeder;

  /* <--------------------------------------------------------------------------------------------------------------------> */

  public RobotContainer() {
    if(Robot.isReal()){
      realSetup();
    }
    else{
      simSetup();
    }

    configureNamedCommands();
    configureBindings();
  }
  
  /* <--------------------------------------------------------------------------------------------------------------------> */

  private void realSetup() {
    this.drivetrain = new DrivetrainSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    this.intake = new Intake(new IntakeIOReal());
    this.shooter = new Shooter(new ShooterIOReal());
    this.extension = new Extension(new ExtensionIOReal());
    this.feeder = new Feeder(new FeederIOReal());
    this.vision = new Vision(new VisionIOReal(VisionConstants.CAMERA_NAME, VisionConstants.CAMERA_NAME2));
  }
  
  /* <--------------------------------------------------------------------------------------------------------------------> */

  private void simSetup() {
    this.drivetrain = new DrivetrainSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    this.intake = new Intake(new IntakeIOSim(drivetrain.getSwerveDriveSim()));
    this.shooter = new Shooter(new ShooterIOSim(drivetrain.getSwerveDriveSim()));
    this.visionSim = new VisionSystemSim("GlobalVisionSim");
    this.feeder = new Feeder(new FeederIOSim());
    this.extension = new Extension(new ExtensionIOSim(drivetrain.getSwerveDriveSim()));
    VisionIOSim cameraSim = new VisionIOSim(visionSim, "MainCamera");
    this.vision = new Vision(cameraSim);

    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /* <--------------------------------------------------------------------------------------------------------------------> */

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        new FieldRelativeDriveCommand(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            () -> -driverController.getRightX()));

    // Gyro Sıfırlama (OPTIONS)
    options.onTrue(Commands.runOnce(drivetrain::zeroGyro));

    // Aim At Tag (PS)
    ps.toggleOnTrue(new AimAtTagCommand(
        drivetrain,
        vision,
        () -> -driverController.getRightX() * RobotConstants.MAX_SPEED,
        () -> -driverController.getLeftY() * RobotConstants.MAX_SPEED,
        () -> -driverController.getLeftX() * RobotConstants.MAX_SPEED
    ));

    // Feeder (L1)
    l3.whileTrue(new RunCommand(() -> feeder.setOutput(0.5), feeder))
       .onFalse(new RunCommand(() -> feeder.setOutput(0), feeder));
 
    // Intake fuel (L2)
    l2.whileTrue(new RunCommand(() -> intake.intakeOutput(0.7), intake))
       .onFalse(new RunCommand(() -> intake.intakeOutput(0), intake));

    // Ters intake (Cross)
    cross.whileTrue(new RunCommand(() -> intake.intakeOutput(-0.7), intake))
       .onFalse(new RunCommand(() -> intake.intakeOutput(0), intake));

    // Open Intake (Circle)
    circle.whileTrue(new RunCommand(() -> extension.extensionOutPut(0.7), extension))
       .onFalse(new RunCommand(() -> extension.extensionOutPut(0), extension));

    // Close Intake (Triangle)
    triangle.whileTrue(new RunCommand(() -> extension.extensionOutPut(-0.7), extension))
       .onFalse(new RunCommand(() -> extension.extensionOutPut(0), extension));
    
    // Shoot (square) — AprilTag görünürken mesafeye göre targetRPS (yakın düşük, uzak yüksek)
    square.whileTrue(new RunCommand(() -> {
      vision.getBestTargetDistanceMeters().ifPresent(dist -> {
        Logger.log("Vision/Shoot/TagDistanceM", dist);
        shooter.setTargetRps(ShooterConstants.targetRpsForAprilTagDistanceMeters(dist));
      });
      shooter.shoot(true);
      feeder.setOutput(1);
    }, shooter, vision, feeder))
       .onFalse(new RunCommand(() -> {
        shooter.shoot(false);
        feeder.setOutput(0);
      }
       , shooter, feeder));

    // Indexer (r1)
    r3.whileTrue(new RunCommand(() -> shooter.indexerOutput(0.7), shooter))
       .onFalse(new RunCommand(() -> shooter.indexerOutput(0), shooter));

    // Shooter +1 RPS (r3)
    r1.onTrue(new InstantCommand(() -> shooter.increaseSpeed(), shooter));

    // Shooter -1 RPS (l3) 
    l1.onTrue(new InstantCommand(() -> shooter.decreaseSpeed(), shooter));
  }

  /* <--------------------------------------------------------------------------------------------------------------------> */

  public void periodic() {
    if (Robot.isSimulation()) {
      Pose2d realPhysicsPose = drivetrain.getSwerveDriveSim().getSimulatedDriveTrainPose();
      visionSim.update(realPhysicsPose);
      SimulatedArena.getInstance().simulationPeriodic();
      
      Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
      Logger.log("FieldSimulation/Fuels", fuelPoses);
    }
    if(Robot.isReal()) {
      
      double yaw = GetYaw.getHeading();
      Logger.log("RobotYaw", yaw);
    }
  }

  /* <--------------------------------------------------------------------------------------------------------------------> */

  public void configureNamedCommands() {
    NamedCommands.registerCommand("Intake", Commands.startEnd(
        () -> intake.intakeOutput(0.8), () -> intake.intakeOutput(0), intake));

    NamedCommands.registerCommand("Shoot", Commands.run(() -> {
      vision.getBestTargetDistanceMeters().ifPresent(dist ->
          shooter.setTargetRps(ShooterConstants.targetRpsForAprilTagDistanceMeters(dist)));
      shooter.shoot(true);
      feeder.setOutput(0.5);
    }, shooter, vision,feeder));

    NamedCommands.registerCommand("Open Intake", Commands.startEnd(
        () -> extension.extensionOutPut(0.7), () -> extension.extensionOutPut(0), extension));

  }
  
  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("bluemiddle");
  }
}