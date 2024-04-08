// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerConstants;

public class LEDs extends SubsystemBase {
  final double GREEN = .77;
  final double RED = .61;
  final double RAINBOW = -.99;

  private boolean aimingAtShooter = true;

  PWMSparkMax LEDs;

  /** Creates a new LED. */
  public LEDs() {
    LEDs = new PWMSparkMax(MotorControllerConstants.LEDs);
    LEDs.set(GREEN);
  }

  // primed for shooter
  public void makeGreen() {
    LEDs.set(GREEN);
    aimingAtShooter = true;
  }

  // intaking
  public void makeRed() {
    LEDs.set(RED);
    aimingAtShooter = false;
  }

  public void makeRainbow() {
    LEDs.set(RAINBOW);
  }

  public void setColor(double shooterVelocity) {
    if (Math.abs(shooterVelocity) > 5000) {
      makeRainbow();
    } else {
      if (aimingAtShooter) {
        makeGreen();
      } else {
        makeRed();
      }
    }
  }

}
