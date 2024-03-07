// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Robot;

import java.math.BigDecimal;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  MecanumDrive mecanumDrive;
  RelativeEncoder fLEncoder, fREncoder, rLEncoder, rREncoder;
  CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

  AHRS navx;

  boolean fieldOriented = true;

  Pose2d startPose2d = new Pose2d(0, 0, new Rotation2d());
  Pose2d robotPose;

  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics;

  MecanumDriveOdometry m_odometry;

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Drive motors
    frontLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.MotorControllerConstants.frontRight, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.backLeft, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(Constants.MotorControllerConstants.backRight, MotorType.kBrushless);
    
    fLEncoder = frontLeftMotor.getEncoder();
    fREncoder = frontRightMotor.getEncoder();
    rLEncoder = rearLeftMotor.getEncoder();
    rREncoder = rearRightMotor.getEncoder();

    fLEncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    fREncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    rLEncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    rREncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);

    fLEncoder.setVelocityConversionFactor(Constants.OdometryConstants.encoderVelocityConversionFactor);
    fREncoder.setVelocityConversionFactor(Constants.OdometryConstants.encoderVelocityConversionFactor);
    rLEncoder.setVelocityConversionFactor(Constants.OdometryConstants.encoderVelocityConversionFactor);
    rREncoder.setVelocityConversionFactor(Constants.OdometryConstants.encoderVelocityConversionFactor);
    
    resetEncoders();

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Creating drive system
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    mecanumDrive.setSafetyEnabled(false);

    // Gyroscope
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    m_frontLeftLocation = Constants.OdometryConstants.frontLeftLocation;
    m_frontRightLocation = Constants.OdometryConstants.frontRightLocation;
    m_backLeftLocation = Constants.OdometryConstants.backLeftLocation;
    m_backRightLocation = Constants.OdometryConstants.backRightLocation;

    m_kinematics = new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        getAngleRotation2d(),
        new MecanumDriveWheelPositions(
            fLEncoder.getPosition(), fREncoder.getPosition(),
            rLEncoder.getPosition(), rREncoder.getPosition()),
        startPose2d);

    SmartDashboard.putData("Field", m_field);
  }

  public Rotation2d getAngleRotation2d() {
    return navx.getRotation2d();
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        fLEncoder.getPosition(), fREncoder.getPosition(),
        rLEncoder.getPosition(), rREncoder.getPosition());
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        fLEncoder.getVelocity(), fREncoder.getVelocity(),
        rLEncoder.getVelocity(), rREncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my wheel positions
    var wheelPositions = getWheelPositions();

    // Get the rotation of the robot from the gyro.
    var gyroAngle = getAngleRotation2d();
    // Update the pose
    robotPose = m_odometry.update(gyroAngle, wheelPositions);

    printPose(robotPose);
  }

  public void printPose(Pose2d robotPose) {
    BigDecimal x = new BigDecimal(robotPose.getX());
    x.setScale(3);
    double X = x.doubleValue();
    SmartDashboard.putNumber("X:", X);

    BigDecimal y = new BigDecimal(robotPose.getY());
    y.setScale(3);
    double Y = y.doubleValue();
    SmartDashboard.putNumber("Y: ", Y);

    BigDecimal angle = new BigDecimal(MathUtil.angleModulus(robotPose.getRotation().getDegrees()));
    angle.setScale(1);
    double theta = angle.doubleValue();
    SmartDashboard.putNumber("Heading: ", theta);
    
    /*
     * String x = String.format("X: %.3f", robotPose.getX());
     * String y = String.format("Y: %.3f", robotPose.getY());
     * String angle = String.format("Angle: %.1f",
     * MathUtil.angleModulus(robotPose.getRotation().getDegrees()));
     * SmartDashboard.putString("Pose: ", x + "\n" + y + "\n" + angle);
     */

     m_field.setRobotPose(robotPose);
     SmartDashboard.putData("Field", m_field);
  }

  public void resetPose() {
    m_odometry.resetPosition(getAngleRotation2d(), getWheelPositions(), startPose2d);
  }

  public void toggleMode() {
    fieldOriented = !fieldOriented;
  }

  public void resetDegree() {
    navx.reset();
    m_odometry.resetPosition(getAngleRotation2d(), getWheelPositions(), robotPose);
  }

  public void resetEncoders() {
    fLEncoder.setPosition(0);
    fREncoder.setPosition(0);
    rLEncoder.setPosition(0);
    rREncoder.setPosition(0);
  }

  public void drive(double forward, double strafe, double turn) {
    if (fieldOriented) {
      driveFieldOriented(forward, strafe, turn);
    } else
      driveRobotOriented(forward, strafe, turn);
  }

  public void driveFieldOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn, navx.getRotation2d().unaryMinus());
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn);
  }
}
