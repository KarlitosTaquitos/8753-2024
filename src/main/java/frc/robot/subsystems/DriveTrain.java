// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  MecanumDrive mecanumDrive;

  public CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  
  public RelativeEncoder frontLeftEncoder, frontRightEncoder, rearLeftEncoder, rearRightEncoder;
  AHRS navx;

  boolean fieldOriented = true;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Drive motors
    frontLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.MotorControllerConstants.frontRight, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.backLeft, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(Constants.MotorControllerConstants.backRight, MotorType.kBrushless);
    
    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    rearLeftEncoder = rearLeftMotor.getEncoder();
    rearRightEncoder = rearRightMotor.getEncoder();

    resetEncoders();

    
    // Creating drive system
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    mecanumDrive.setSafetyEnabled(false);

    // Gyroscope
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDegree() {
    return navx.getRotation2d().getDegrees(); 
  }

  public void toggleMode() {
    fieldOriented = !fieldOriented;
  }

  public void setToFieldOriented(boolean yes) {
    fieldOriented = yes;
  }

  public void setToBrake() {
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    rearLeftMotor.setIdleMode(IdleMode.kBrake);
    rearRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setToCoast() {
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    rearLeftMotor.setIdleMode(IdleMode.kCoast);
    rearRightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    rearLeftEncoder.setPosition(0);
    rearRightEncoder.setPosition(0);
  }

  public void convertEncoderUnits() {
    frontLeftEncoder.setPositionConversionFactor(Constants.EncoderConstants.distancePerRevolution);
    frontRightEncoder.setPositionConversionFactor(Constants.EncoderConstants.distancePerRevolution);
    rearLeftEncoder.setPositionConversionFactor(Constants.EncoderConstants.distancePerRevolution);
    rearRightEncoder.setPositionConversionFactor(Constants.EncoderConstants.distancePerRevolution);

    frontLeftEncoder.setVelocityConversionFactor(Constants.EncoderConstants.velocityConversionFactor);
    frontRightEncoder.setVelocityConversionFactor(Constants.EncoderConstants.velocityConversionFactor);
    rearLeftEncoder.setVelocityConversionFactor(Constants.EncoderConstants.velocityConversionFactor);
    rearRightEncoder.setVelocityConversionFactor(Constants.EncoderConstants.velocityConversionFactor);
  }

  public void resetDegree() {
    navx.reset();
  }

  public void drive(double forward, double strafe, double turn) {
    // dead zone
    if (forward < 0.05 && forward > -0.05)
      forward = 0;
    if (strafe < 0.05 && strafe > -0.05)
      strafe = 0;
    if (turn < 0.05 && turn > -0.05)
      turn = 0;


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
