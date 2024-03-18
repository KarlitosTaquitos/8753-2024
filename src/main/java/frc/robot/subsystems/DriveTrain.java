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
import frc.robot.Constants.OdometryConstants;

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
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
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
    
// Creating drive system
    mecanumDrive = new MecanumDrive(frontLeftMotor::set, rearLeftMotor::set, frontRightMotor::set, rearRightMotor::set);
    mecanumDrive.setSafetyEnabled(false);

    fLEncoder = frontLeftMotor.getEncoder();
    fREncoder = frontRightMotor.getEncoder();
    rLEncoder = rearLeftMotor.getEncoder();
    rREncoder = rearRightMotor.getEncoder();

    fLEncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    fREncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    rLEncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    rREncoder.setPositionConversionFactor(Constants.OdometryConstants.encoderDistancePerPulse);
    
    resetEncoders();

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Gyroscope
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    m_kinematics = OdometryConstants.mecanumDriveKinematics;

    m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        getAngleRotation2d(),
        new MecanumDriveWheelPositions(),
        startPose2d);

    SmartDashboard.putData("Field", m_field);
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
    SmartDashboard.putNumber("X:", robotPose.getX());

    SmartDashboard.putNumber("Y: ", robotPose.getY());

    SmartDashboard.putNumber("Heading: ", MathUtil.angleModulus(robotPose.getRotation().getDegrees()));

     m_field.setRobotPose(robotPose);
     SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose() {
    return robotPose;
  }

  //@param pose t0 set odometry to
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getAngleRotation2d(), getWheelPositions(), pose);
  }

  public void toggleMode() {
    fieldOriented = !fieldOriented;
  }

  public void drive(double forward, double strafe, double turn) {
    if (fieldOriented) {
      driveFieldOriented(forward, strafe, turn);
    } else
      driveRobotOriented(forward, strafe, turn);
  }

  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    frontLeftMotor.setVoltage(volts.frontLeftVoltage);
    rearLeftMotor.setVoltage(volts.rearLeftVoltage);
    frontRightMotor.setVoltage(volts.frontRightVoltage);
    rearRightMotor.setVoltage(volts.rearRightVoltage);
  }

  public void resetEncoders() {
    fLEncoder.setPosition(0);
    fREncoder.setPosition(0);
    rLEncoder.setPosition(0);
    rREncoder.setPosition(0);
  }

  public RelativeEncoder getfLEncoder() {
    return fLEncoder;
  }

  public RelativeEncoder getfREncoder() {
    return fREncoder;
  }

  public RelativeEncoder getrLEncoder() {
    return rLEncoder;
  }

  public RelativeEncoder getrREncoder() {
    return rREncoder;
  }

  public Rotation2d getAngleRotation2d() {
    return navx.getRotation2d();
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        fLEncoder.getVelocity(), fREncoder.getVelocity(),
        rLEncoder.getVelocity(), rREncoder.getVelocity());
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        fLEncoder.getPosition(), fREncoder.getPosition(),
        rLEncoder.getPosition(), rREncoder.getPosition());
  }

  //set max power to set Drivetrain
  public void setMaxOutput(double maxOutpu) {
    mecanumDrive.setMaxOutput(maxOutpu);
  }

  public void resetDegree() {
    navx.reset();
  }

  public double getTurnRate() {
    return navx.getRate();
  }

  public void driveFieldOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn, navx.getRotation2d().unaryMinus());
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn);
  }
}
