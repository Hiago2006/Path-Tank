// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.DriveWithJoystickCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();

  private final DriveWithJoystickCommand driveWithJoystickCommand = new DriveWithJoystickCommand(driveTrainSubsystem);

  public static XboxController joystick = new XboxController(0);

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveTrainSubsystem.setDefaultCommand(driveWithJoystickCommand);

    chooser.addOption("New Path",
        loadPathplannerTrajectoryToRamseteCommand("pathplanner/generatedJSON/New Path.wpilib.json", true));

    Shuffleboard.getTab("Auto").add(chooser);
  }

  public Command loadPathplannerTrajectoryToRamseteCommand(String filename, boolean resetOdomtry) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Nao pode abrir a trajetoria" + filename, exception.getStackTrace());
      System.out.println("Nao pode ler o arquivo" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrainSubsystem::getPose,
        new RamseteController(DriveTrainConstants.kRamseteB, DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeters,
            DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics, driveTrainSubsystem::getWheelSpeeds,
        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0),
        new PIDController(DriveTrainConstants.kpDriveVel, 0, 0), driveTrainSubsystem::tankDriveVolts,
        driveTrainSubsystem);

    if (resetOdomtry) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> driveTrainSubsystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
    } else {
      return ramseteCommand;
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
