// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooter;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    shooter = new CANSparkMax(Constants.MotorControllerConstants.shooter, MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void runShooter() {
    shooter.set(1);
  }

  public void stopShooter() {
    shooter.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
