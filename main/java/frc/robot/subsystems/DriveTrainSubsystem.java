// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class DriveTrainSubsystem extends SubsystemBase {

  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.DriveTrainConstants.leftFrontCANID,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.DriveTrainConstants.leftBackCANID,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.DriveTrainConstants.rightFrontCANID,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.DriveTrainConstants.rightBackCANID,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftBackMotor, leftFrontMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightBackMotor, rightFrontMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public static final Pigeon2 pigeon = new Pigeon2(5);
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new ExampleSubsystem. */
  public DriveTrainSubsystem() {
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);
    leftEncoder.setVelocityConversionFactor(DriveTrainConstants.kLinearDistanceConversionFactor / 60);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);

    pigeon.setAccumZAngle(0, 0);
    pigeon.setYaw(0, 0);

    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeon.getYaw()), leftEncoder.getPosition(),
        rightEncoder.getPosition());
    m_odometry.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw()), getLeftEncoderPosition(),
        getRightEncoderPosition(), new Pose2d());
    setBreakMode();
  }

  public void setBreakMode() {
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public double getRightEncoderPosition() {
    return -rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }

  public static double getHeading() {
    return pigeon.getYaw();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw()), getLeftEncoderPosition(),
        getRightEncoderPosition(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition() / 2);
  }

  public RelativeEncoder getLefEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public static void zeroHeading() {
    pigeon.setAccumZAngle(0, 0);
    pigeon.setYaw(0, 0);
  }

  public Gyro getGyro() {
    return getGyro();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(pigeon.getYaw()), leftEncoder.getPosition(),
        rightEncoder.getPosition());

    SmartDashboard.putNumber("Left encoder value meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value meters", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro heading", getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
