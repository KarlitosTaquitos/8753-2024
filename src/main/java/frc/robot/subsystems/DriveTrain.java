// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  MecanumDrive mecanumDrive;

  CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

  AHRS navx;

  TurnState turnState = TurnState.WAITING;

  double oldAngle;

  int quadrant;

  final double TURN_TOLERANCE = 2.0;

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

  public void toggleMode() {
    fieldOriented = !fieldOriented;
  }
  
  public void resetDegree(){
    navx.reset();
  }
  //returns angle in degrees. degree is continuous so once it hits 360 it goes to 361 and so on
  public double getAngle() {
    return navx.getRotation2d().getDegrees();
  }

  public void drive(double forward, double strafe, double turn, boolean rightBumper, boolean leftBumper) {
    if (fieldOriented) {
      switch(turnState) {
        case WAITING:
          driveFieldOriented(forward, strafe, turn);

          if(rightBumper) {
            oldAngle = getAngle();
            quadrant = ((int)(getAngle() / 90)) % 4;

            turnState = TurnState.TURNING_RIGHT;
          }
          if(leftBumper) {
            oldAngle = getAngle();
            quadrant = ((int)(getAngle() / 90)) % 4;

            turnState = TurnState.TURNING_LEFT;
          }
          break;
        case TURNING_RIGHT:
          driveFieldOriented(forward, strafe, .5);
          
          switch(quadrant) {
            case 0:
              if(getAngle() < TURN_TOLERANCE && getAngle() > -TURN_TOLERANCE) {
                turnState = TurnState.WAITING;
              }
              break;
            case 1:
              if(getAngle() < TURN_TOLERANCE + 90 && getAngle() > -TURN_TOLERANCE + 90) {
                turnState = TurnState.WAITING;
              }
              break;
            case 2:
              if(getAngle() < TURN_TOLERANCE + 180 && getAngle() > -TURN_TOLERANCE + 180) {
                turnState = TurnState.WAITING;
              }
              break;
            case 3:
              if(getAngle() < TURN_TOLERANCE + 270 && getAngle() > -TURN_TOLERANCE + 270) {
                turnState = TurnState.WAITING;
              }
              break;
          }
          break;
        case TURNING_LEFT:
          driveFieldOriented(forward, strafe, -.5);
          
          switch(quadrant) {
            case 0:
              if(getAngle() < TURN_TOLERANCE + 90 && getAngle() > -TURN_TOLERANCE + 90) {
                turnState = TurnState.WAITING;
              }
              break;
            case 1:
              if(getAngle() < TURN_TOLERANCE + 180 && getAngle() > -TURN_TOLERANCE + 180) {
                turnState = TurnState.WAITING;
              }
              break;
            case 2:
              if(getAngle() < TURN_TOLERANCE + 270 && getAngle() > -TURN_TOLERANCE + 270) {
                turnState = TurnState.WAITING;
              }
              break;
            case 3:
              if(getAngle() < TURN_TOLERANCE && getAngle() > -TURN_TOLERANCE) {
                turnState = TurnState.WAITING;
              }
              break;
          }
          break;
        }
    } else
      driveRobotOriented(forward, strafe, turn);
      
  }

  public void driveFieldOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn, navx.getRotation2d().unaryMinus());
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn);
  }
  private enum TurnState {
    WAITING,
    TURNING_LEFT,
    TURNING_RIGHT
  }
}
