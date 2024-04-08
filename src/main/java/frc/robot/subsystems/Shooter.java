// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax topShooter;
  CANSparkMax middleShooter;
  CANSparkMax bottomShooter;

  RelativeEncoder lShooterEncoder;
  RelativeEncoder rShooterEncoder;
  RelativeEncoder bottomEncoder;

  /** Creates a new Intake. */
  public Shooter() {
    topShooter = new CANSparkMax(MotorControllerConstants.topShooter, MotorType.kBrushless);
    topShooter.setIdleMode(IdleMode.kCoast);

    middleShooter = new CANSparkMax(MotorControllerConstants.middleShooter, MotorType.kBrushless);
    middleShooter.setIdleMode(IdleMode.kCoast);

    bottomShooter = new CANSparkMax(MotorControllerConstants.bottomShooter, MotorType.kBrushless);
    bottomShooter.setIdleMode(IdleMode.kCoast);

    lShooterEncoder = topShooter.getEncoder();
    rShooterEncoder = middleShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();
  }

  public void shootSpeaker() {
    topShooter.setVoltage(-12);
    middleShooter.setVoltage(12);
  }

  public void shootOver(){
    topShooter.setVoltage(-10);
    middleShooter.setVoltage(10);
  }

  public void shootAmp() {
    topShooter.setVoltage(-1.7);
    middleShooter.setVoltage(3);
  }

  public void shootThrough() {
    middleShooter.setVoltage(-12);
    bottomShooter.setVoltage(12);
  }

  public void stop() {
    topShooter.setVoltage(0);
    middleShooter.setVoltage(0);
    bottomShooter.setVoltage(0);
  }

  public double getShooterVelocity() {
    return rShooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity Left", lShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Velocity Right", rShooterEncoder.getVelocity());
  }
}
