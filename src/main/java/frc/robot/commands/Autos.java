// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
    return new SequentialCommandGroup(
      new RunCommand(() -> s.shoot(), s).withTimeout(.25),
      new RunCommand(() -> i.outtake(), i).withTimeout(.75),  
      new ParallelCommandGroup(
        new RunCommand(() -> i.stopIntake(), i).withTimeout(.25),
        new RunCommand(() -> s.stop(), s).withTimeout(.25)
      ));
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

  public static Command testAutoDrive(DriveTrain dt) {
    return new DriveAmount(-0.3, -.2, -.1, dt, 1.5);

  }

  //Start Pose in front of amp
  public static Command startInMiddleTwoNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      shootNote(i, s),
      new ParallelCommandGroup(
        new DriveAmount(-.3, 0, 0, dt, 1.5),
        new SequentialCommandGroup(
          new MoveIntakeToFloor(i).withTimeout(1),
          new RunCommand(() -> i.intake(), i).withTimeout(1)
        )),
      new ParallelCommandGroup(
        new DriveAmount(.3, 0, 0, dt, 1.5),
        new SequentialCommandGroup(
          new RunCommand(() -> i.stopIntake(), i).withTimeout(1),
          new MoveIntakeInside(i).withTimeout(1)
        )),
      shootNote(i, s)
    );
  }

   public static Command startInMiddleLeftNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new DriveAmount(-.4, -.4, -.35, dt, .75),
          new DriveAmount(-.4, 0, 0, dt, .75)),
        new SequentialCommandGroup(
          new MoveIntakeToFloor(i).withTimeout(1),
          new RunCommand(() -> i.intake(), i).withTimeout(1.75)
        )),
      new ParallelCommandGroup(
        new DriveAmount(.3, .3, .1, dt, 1.5),
        new SequentialCommandGroup(
          new RunCommand(() -> i.stopIntake(), i).withTimeout(.5),
          new MoveIntakeInside(i).withTimeout(1)
        )),
      new DriveAmount(.2, 0, 0, dt, 1),
      shootNote(i, s)
    );
  }

  public static Command startInMiddleRightNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new DriveAmount(-.4, .4, .35, dt, .75),
          new DriveAmount(-.4, 0, 0, dt, .75)),
        new SequentialCommandGroup(
          new MoveIntakeToFloor(i).withTimeout(1),
          new RunCommand(() -> i.intake(), i).withTimeout(1.75)
        )),
      new ParallelCommandGroup(
        new DriveAmount(.3, -.3, -.1, dt, 1.5),
        new SequentialCommandGroup(
          new RunCommand(() -> i.stopIntake(), i).withTimeout(.5),
          new MoveIntakeInside(i).withTimeout(1)
        )),
      new DriveAmount(.2, 0, 0, dt, 1),
      shootNote(i, s)
    );
  }
  
  public static Command startInMiddleThreeNoteBlue(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(startInMiddleTwoNote(dt, i, s),
    startInMiddleLeftNote(dt, i, s));
  }

  public static Command startInMiddleThreeNoteRed(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(startInMiddleTwoNote(dt, i, s),
    startInMiddleRightNote(dt, i, s));
  }
  //Start right of amp. Should work on both sides
  public static Command startRightTwoOne(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      new DriveAmount(0, .2, 0, dt, 2).withTimeout(2),
      shootNote(i, s),
      new DriveAmount(0, -.2, 0, dt, 2).withTimeout(2));
  }

  //Start Left of amp. Both sides
  public static Command startRightOneNote(DriveTrain dt, Intake i, Shooter s) {
    return new SequentialCommandGroup(
      shootNote(i, s),
      new DriveAmount(-.3, 0, 0, dt, 4).withTimeout(6));
  }
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
