// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  public static final double robotLength = 1;
  public static final double robotWidth = .8;

  // TODO: Fill these
  public final class StartPoses {
    public static final Pose2d SPEAKER_MIDDLE_RED = new Pose2d();
    public static final Pose2d SPEAKER_MIDDLE_BLUE = new Pose2d();
    public static final Pose2d SPEAKER_AMP_RED = new Pose2d();
    public static final Pose2d SPEAKER_AMP_BLUE = new Pose2d();
    public static final Pose2d SPEAKER_SOURCE_RED = new Pose2d();
    public static final Pose2d SPEKAER_SOURCE_BLUE = new Pose2d();
  }


  //---------------------------------- Trajectories -----------------------------//

  // An example trajectory to follow. All units in meters.
  //This trajectory follows an S shape
  public static Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      AutoConstants.trajectoryConfig);
  
  //this one turns the robot around
  public static Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
    new Pose2d(3,0,new Rotation2d(0)), //end of example trajectory
    List.of(new Translation2d(4,0)),
    new Pose2d(3,1,new Rotation2d(Math.toRadians(180))),
    AutoConstants.trajectoryConfig);

  //--------------------------------------- Commands to Run -------------------------//

  //This method returns a command that first resets odometry and puts start postition
  //Then we run the path
  //Then tell the drive to stop
  public static Command exampleRun(DriveTrain dt) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> dt.resetOdometry(exampleTrajectory.getInitialPose())),
        followPath(exampleTrajectory, dt),
        new InstantCommand(() -> dt.drive(0, 0, 0)));
  }

  //This method returns a command that tries to run two trajectories back to back
  public static Command twoTrajectoryRun(DriveTrain dt) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> dt.resetOdometry(exampleTrajectory.getInitialPose())),
        followPath(exampleTrajectory, dt),
        followPath(exampleTrajectory2, dt),
        new InstantCommand(() -> dt.drive(0, 0, 0)));
  }
  
  /*
   * Method to take in a trajectory for a drivetrain to follow when command is
   * activated
   * May or may not work lol
   * This will help to ensure the same constants and everything are used on every path followed
   * USE FOR ALL FOLLOWING
   */
  public static MecanumControllerCommand followPath(Trajectory traj, DriveTrain dt) {
    return new MecanumControllerCommand(
        traj,
        dt::getPose,
        AutoConstants.kFeedforward,
        OdometryConstants.mecanumDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

        // Needed for normalizing wheel speeds
        AutoConstants.kMaxSpeedMetersPerSecond,

        // Velocity PID's
        new PIDController(AutoConstants.kPFrontLeftVel, 0, 0),
        new PIDController(AutoConstants.kPRearLeftVel, 0, 0),
        new PIDController(AutoConstants.kPFrontRightVel, 0, 0),
        new PIDController(AutoConstants.kPRearRightVel, 0, 0),
        dt::getWheelSpeeds,
        dt::setDriveMotorControllersVolts, // Consumer for the output motor voltages
        dt);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
