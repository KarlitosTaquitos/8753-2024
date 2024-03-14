// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveAmount extends Command {
  private DriveTrain driveTrain;
  private double forward, strafe, turn;
  private double initialDegree;

  // If internal timer doesn't work, we run command with a .withTimeout
  private double timeToLive;
  private Timer timer = new Timer();

  private boolean timerStarted = false;

  /** Creates a new DriveAmount. */
  public DriveAmount(double fowardSpeed, double strafeSpeed, double turnSpeed, DriveTrain dt, double timeout) {
    driveTrain = dt;
    forward = fowardSpeed;
    strafe = strafeSpeed;
    turn = turnSpeed;

    timeToLive = timeout;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* 
    if(turn == 0) {
      initialDegree = driveTrain.getDegree();
    }
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!timerStarted) {
      timerStarted = true;
      timer.restart();
    }

    driveTrain.drive(forward, strafe, turn);
      /* 
    if(turn == 0) {
      if(initialDegree > driveTrain.getDegree()) {
        
      }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeToLive);
  }
}
