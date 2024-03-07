// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // TODO: Verify ports and button mappings
  public static class DriverConstants {
    public static final int controllerPort = 0;
    public static final int leftX = 0;
    public static final int leftY = 1;
    public static final int rightX = 4;
    public static final int rightY = 5;

    public static final int a = 1;

    public static final int start = 8;
    public static final int back = 7;

    public static final double driveMult = 0.25;
  }

  public static class OperatorConstants {
    public static final int controllerPort = 1;
  }

  public static class MotorControllerConstants {
    public static final int frontLeft = 1;
    public static final int frontRight = 2;
    public static final int backLeft = 3;
    public static final int backRight = 4;
  }
  //All meters
  public static class OdometryConstants {
    public static final Translation2d frontLeftLocation = new Translation2d(0.2604, 0.2604);
    public static final Translation2d frontRightLocation = new Translation2d(0.2604, -0.2604);
    public static final Translation2d backLeftLocation = new Translation2d(-0.2604, 0.2604);
    public static final Translation2d backRightLocation = new Translation2d(-0.2604, -0.2604);

    public static final double wheelRadius = .1016;
    public static final double wheelCircumference = wheelRadius * 2 * Math.PI;
    public static final double chassisGearRatio = 12.5;
    public static final double encoderDistancePerPulse = wheelCircumference / chassisGearRatio;
    public static final double encoderVelocityConversionFactor = encoderDistancePerPulse * 60;
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

    public static final double kPDriveVel = 0;
  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
