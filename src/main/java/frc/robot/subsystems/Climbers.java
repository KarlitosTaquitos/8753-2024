// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;

public class Climbers extends SubsystemBase {
  public static class Positions {
    public static final int lBottom = -2;
    public static final int lTop = 195;

    public static final int rBottom = 2;
    public static final int rTop = -195;
  }

  private CANSparkMax leftClimber = new CANSparkMax(MotorControllerConstants.lClimber, MotorType.kBrushless);
  private CANSparkMax rightClimber = new CANSparkMax(MotorControllerConstants.rClimber, MotorType.kBrushless);

  private RelativeEncoder lEncoder = leftClimber.getEncoder();
  private RelativeEncoder rEncoder = rightClimber.getEncoder();

  private boolean activeLimit = true;

  /** Creates a new Climbers. */
  public Climbers() {
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  }

  public void runClimbers(double left, double right) {
    left *= -1;

    if (activeLimit) {
      // Check direction, + up, - down and set power accordingly
      if (left > 0) {
        if (lEncoder.getPosition() < Positions.lTop)
          leftClimber.set(left * 0.5);
        else
          leftClimber.set(0);
      } else {
        if (lEncoder.getPosition() > Positions.lBottom)
          leftClimber.set(left * 0.5);
        else
          leftClimber.set(0);
      }

      if (right > 0) {
        if (rEncoder.getPosition() < Positions.rBottom)
          rightClimber.set(right * 0.5);
        else
          rightClimber.set(0);
      } else {
        if (rEncoder.getPosition() > Positions.rTop)
          rightClimber.set(right * 0.5);
        else
          rightClimber.set(0);
      }
    } else {
      leftClimber.set(left * 0.5);
      rightClimber.set(right * 0.5);
    }
  }

  public void toggleLimit() {
    activeLimit = !activeLimit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
