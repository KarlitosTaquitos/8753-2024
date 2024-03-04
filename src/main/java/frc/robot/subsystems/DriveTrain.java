// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  MecanumDrive mecanumDrive;
  RelativeEncoder fLEncoder, fREncoder, rLEncoder, rREcnoder;
  CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

  AHRS navx;

  boolean fieldOriented = true;

  Pose2d robotPose = new Pose2d(0, 0, new Rotation2d());

  Translation2d m_frontLeftLocation;
  Translation2d m_frontRightLocation;
  Translation2d m_backLeftLocation;
  Translation2d m_backRightLocation;

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics;

  MecanumDriveOdometry m_odometry;

  SysIdRoutine sysIdRoutine;

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Drive motors
    frontLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.MotorControllerConstants.frontRight, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(Constants.MotorControllerConstants.backLeft, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(Constants.MotorControllerConstants.backRight, MotorType.kBrushless);

    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);

    fLEncoder = frontLeftMotor.getEncoder();
    fREncoder = frontRightMotor.getEncoder();
    rLEncoder = rearLeftMotor.getEncoder();
    rREcnoder = rearRightMotor.getEncoder();

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    // Creating drive system
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    mecanumDrive.setSafetyEnabled(false);

    // Gyroscope
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    m_frontLeftLocation = Constants.OdometryConstants.frontLeftLocation;
    m_frontRightLocation = Constants.OdometryConstants.frontRightLocation;
    m_backLeftLocation = Constants.OdometryConstants.backLeftLocation;
    m_backRightLocation = Constants.OdometryConstants.backRightLocation;

    m_kinematics = new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    m_odometry = new MecanumDriveOdometry(
        m_kinematics,
        getAngleRotation2d(),
        new MecanumDriveWheelPositions(
            fLEncoder.getPosition(), fREncoder.getPosition(),
            rLEncoder.getPosition(), rREcnoder.getPosition()),
        robotPose);

    sysIdRoutine = new SysIdRoutine(
        new Config(),
        new Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
              frontLeftMotor.setVoltage(volts.in(Volts));
              frontRightMotor.setVoltage(volts.in(Volts));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being
            // characterized.
            log -> {
              // Record a frame for the left motors. Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          frontLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(fLEncoder.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(fLEncoder.getVelocity(), MetersPerSecond));
              // Record a frame for the right motors. Since these share an encoder, we
              // consider
              // the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(
                      m_appliedVoltage.mut_replace(
                          frontRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(fREncoder.getPosition(), Meters))
                  .linearVelocity(m_velocity.mut_replace(fREncoder.getVelocity(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in
            // WPILog with this subsystem's name ("DriveTrain")
            this));

  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Rotation2d getAngleRotation2d() {
    return navx.getRotation2d().unaryMinus();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my wheel positions
    var wheelPositions = new MecanumDriveWheelPositions(
        fLEncoder.getPosition(), fREncoder.getPosition(),
        rLEncoder.getPosition(), rREcnoder.getPosition());

    // Get the rotation of the robot from the gyro.
    var gyroAngle = getAngleRotation2d();

    // Update the pose
    robotPose = m_odometry.update(gyroAngle, wheelPositions);

    printPose(robotPose);
  }

  public void printPose(Pose2d robotPose) {
    String x = String.format("X: %.2f", robotPose.getX());
    String y = String.format("Y: %.2f", robotPose.getY());
    String angle = String.format("Angle: %.2f", MathUtil.angleModulus(robotPose.getRotation().getDegrees()));
    System.out.println(x + "\n" + y + "\n" + angle);
  }

  public void toggleMode() {
    fieldOriented = !fieldOriented;
  }

  public void resetDegree() {
    navx.reset();
  }

  public void drive(double forward, double strafe, double turn) {
    if (fieldOriented) {
      driveFieldOriented(forward, strafe, turn);
    } else
      driveRobotOriented(forward, strafe, turn);
  }

  public void driveFieldOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn, navx.getRotation2d().unaryMinus());
  }

  public void driveRobotOriented(double forward, double strafe, double turn) {
    mecanumDrive.driveCartesian(-forward, strafe, turn);
  }
}
