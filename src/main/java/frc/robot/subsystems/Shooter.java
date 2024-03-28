// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.w3c.dom.ls.LSProgressEvent;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax lShooter;
  CANSparkMax rShooter;

  RelativeEncoder lShooterEncoder;
  RelativeEncoder rShooterEncoder;

  /** Creates a new Intake. */
  public Shooter() {
    lShooter = new CANSparkMax(MotorControllerConstants.lShooter, MotorType.kBrushless);
    lShooter.setIdleMode(IdleMode.kCoast);
    rShooter = new CANSparkMax(MotorControllerConstants.rShooter,MotorType.kBrushless);
    rShooter.setIdleMode(IdleMode.kCoast);

    lShooterEncoder = lShooter.getEncoder();
    rShooterEncoder = rShooter.getEncoder();
    
  }

  public void shoot() {
    lShooter.setVoltage(-12);
    rShooter.setVoltage(12);
  }

  public void stop() {
    lShooter.setVoltage(0);
    rShooter.setVoltage(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity Left", lShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Velocity Right", rShooterEncoder.getVelocity());
  }
}
