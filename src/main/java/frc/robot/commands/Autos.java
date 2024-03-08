// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }
  public static Command shootNote(Intake i, Shooter s) {
    return new ParallelCommandGroup(
        new RunCommand(() -> i.outtake(), i).withTimeout(2),
        new RunCommand(() -> s.shoot(), s).withTimeout(2));
  }

  public static Command intakeProcedure(DriveTrain dt, Intake i) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DriveAmount(.15, 0, 0, dt, 1),
            new RunCommand(() -> i.intake(), i).withTimeout(1)),
        new ParallelCommandGroup(
            new DriveAmount(-.15, 0, 0, dt, 1),
            new RunCommand(() -> i.intake(), i).withTimeout(1)));
  }

  public static Command testAutoDrive(DriveTrain dt, Intake i, Shooter s) {
    return new DriveAmount(0.2, 0, 0, dt, 1);

  }

  public static Command testAutoDriveAndPickup(DriveTrain dt, Intake i, Shooter s) {
    return new ParallelCommandGroup(
        new DriveAmount(0.2, 0, 0, dt, 5),
        new SequentialCommandGroup(
            new MoveIntakeToFloor(i).withTimeout(3),
            new RunCommand(() -> i.intake(), i).withTimeout(4),
            new MoveIntakeInside(i).withTimeout(3)));
  }

  //Start Pose in front of amp
  public static Command startInMiddleTwoNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      shootNote(i, s),
      new DriveAmount(.4, 0, 0, dt, 4),
      intakeProcedure(dt, i),
      new DriveAmount(-.4, 0, 0, dt, 4),
      shootNote(i, s)
    );
  }

  public static Command startRightTwoNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      new DriveAmount(0, 0, 0, dt, 0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
