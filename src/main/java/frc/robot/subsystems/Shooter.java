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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;
import frc.robot.Constants.SensorConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax lShooter;
  CANSparkMax rShooter;


  /** Creates a new Intake. */
  public Shooter() {
    lShooter = new CANSparkMax(MotorControllerConstants.lShooter, MotorType.kBrushless);
    lShooter.setIdleMode(IdleMode.kCoast);
    rShooter = new CANSparkMax(MotorControllerConstants.rShooter,MotorType.kBrushless);
    rShooter.setIdleMode(IdleMode.kCoast);
  }

  public void shoot() {
    lShooter.set(-1);
    rShooter.set(1);
  }

  public void stop() {
    lShooter.set(0);
    rShooter.set(0);
  }
}
