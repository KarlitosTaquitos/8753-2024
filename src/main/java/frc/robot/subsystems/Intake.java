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

  CANSparkMax intakeMotor;

  final int inside = 0;
  final int amp = -15;
  final int floor = -43;

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
    movementController.setOutputRange(-0.4, 0.6);

    intakeMotor = new CANSparkMax(MotorControllerConstants.intakeMotor, MotorType.kBrushless);

    disablePID();
  }

  public void intake() {
    intakeMotor.set(.75);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void outtake() {
    intakeMotor.set(-.75);
  }

  public void moveDown() {
    movementController.setReference(floor, ControlType.kPosition);
    targetPosition = floor;
  }

  public void moveIntoRobot() {
    movementController.setReference(inside, ControlType.kPosition);
    targetPosition = inside;
  }

  public void moveToAmp() {
    movementController.setReference(amp, ControlType.kPosition);
    targetPosition = amp;
  }

  public void enablePID() {
    movementController.setP(0.1);
    movementController.setD(10);
  }

  public void disablePID() {
    movementController.setP(0);
    movementController.setD(0);
  }

  public boolean isAtSetpoint() {
    switch (targetPosition) {
      case inside:
        return movementEncoder.getPosition() > inside + 2;
      case amp:
        return movementEncoder.getPosition() < amp + 2 && movementEncoder.getPosition() > amp - 2;
      case floor:
        return movementEncoder.getPosition() < floor + 2;
      default:
        return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
