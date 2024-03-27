// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OdometryConstants;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public final class Autos {
  public static final double robotLength = 1;
  public static final double robotWidth = .8;
  public final class StartPoses {
    public static final Pose2d SPEAKER_MIDDLE_RED = new Pose2d();
    public static final Pose2d SPEAKER_MIDDLE_BLUE = new Pose2d();
    public static final Pose2d SPEAKER_AMP_RED = new Pose2d();
    public static final Pose2d SPEAKER_AMP_BLUE = new Pose2d();
    public static final Pose2d SPEAKER_SOURCE_RED = new Pose2d();
    public static final Pose2d SPEKAER_SOURCE_BLUE = new Pose2d();
  }
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            AutoConstants.trajectoryConfig);

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
