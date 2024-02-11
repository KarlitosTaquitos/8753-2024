// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;

public class Intake extends SubsystemBase {
  CANSparkMax movementMotor;
  SparkPIDController movementController;
  RelativeEncoder movementEncoder;

  int targetPosition;

  /** Creates a new Intake. */
  public Intake() {
    movementMotor = new CANSparkMax(MotorControllerConstants.intakeMovement, MotorType.kBrushless);
    movementMotor.setIdleMode(IdleMode.kCoast);

    movementEncoder = movementMotor.getEncoder();
    movementController = movementMotor.getPIDController();

    movementController.setP(0);
    movementController.setI(0);
    movementController.setD(0);
    movementController.setFF(0);
    movementController.setOutputRange(-0.2, 0.3);

    disablePID();
  }

  public void moveDown() {
    movementController.setReference(-33, ControlType.kPosition);
    targetPosition = -33;
  }

  public void moveIntoRobot() {
    movementController.setReference(0, ControlType.kPosition);
    targetPosition = 0;
  }

  public void moveToAmp() {
    movementController.setReference(-15, ControlType.kPosition);
    targetPosition = -15;
  }

  public void enablePID() {
    movementController.setP(0.1);
    movementController.setD(40);
  }

  public void disablePID() {
    movementController.setP(0);
    movementController.setD(0);
  }

  public boolean isAtSetpoint() {
    switch (targetPosition) {
      case 0:
        return movementEncoder.getPosition() > 2;
      case -15:
        return movementEncoder.getPosition() < -12 && movementEncoder.getPosition() > -17;
      case -33:
        return movementEncoder.getPosition() < -30;
      default:
        return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
