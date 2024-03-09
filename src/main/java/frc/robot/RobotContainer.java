// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveIntakeInside;
import frc.robot.commands.MoveIntakeToAmp;
import frc.robot.commands.MoveIntakeToFloor;
import frc.robot.commands.ResetDegree;
import frc.robot.commands.SetToBrake;
import frc.robot.commands.SetToCoast;
import frc.robot.commands.ToggleClimberLimit;
import frc.robot.commands.ToggleDrivingMode;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climbers climbers = new Climbers();

  private final XboxController driver = new XboxController(DriverConstants.controllerPort);
  private final XboxController operator = new XboxController(OperatorConstants.controllerPort);

  // Commands
  private final ToggleDrivingMode toggledriveMode = new ToggleDrivingMode(driveTrain);
  private final ResetDegree resetdegree = new ResetDegree(driveTrain);

  private final MoveIntakeToFloor moveIntakeToFloor = new MoveIntakeToFloor(intake);
  private final MoveIntakeInside moveIntakeInside = new MoveIntakeInside(intake);
  private final MoveIntakeToAmp moveIntakeToAmp = new MoveIntakeToAmp(intake);

  private final ToggleClimberLimit toggleClimberLimit = new ToggleClimberLimit(climbers);

  // Buttons
  // driver
  private final JoystickButton driverLB = new JoystickButton(driver, DriverConstants.lB);
  private final JoystickButton driverStart = new JoystickButton(driver, DriverConstants.start);
  private final JoystickButton driverBack = new JoystickButton(driver, DriverConstants.back);

  // operator
  private final JoystickButton operatorA = new JoystickButton(operator, OperatorConstants.a);
  private final JoystickButton operatorB = new JoystickButton(operator, OperatorConstants.b);
  private final JoystickButton operatorY = new JoystickButton(operator, OperatorConstants.y);
  private final JoystickButton operatorLB = new JoystickButton(operator, OperatorConstants.lB);
  private final JoystickButton operatorRB = new JoystickButton(operator, OperatorConstants.rB);
  private final JoystickButton operatorStart = new JoystickButton(operator, OperatorConstants.start);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * XboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.XboxController Flight
   * joysticks}.
   */
  private void configureBindings() {
    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // // pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    operatorA.onTrue(moveIntakeToFloor);
    operatorB.onTrue(moveIntakeInside);
    operatorY.onTrue(moveIntakeToAmp);

    { // Intake in
      operatorLB
          .onTrue(
              new RunCommand(() -> {
                intake.intake();
              }, intake))
          .onFalse(
              new InstantCommand(() -> {
                intake.stopIntake();
              }, intake));
    }

    { // Intake out
      operatorRB
          .onTrue(
              new RunCommand(() -> {
                intake.outtake();
              }, intake))
          .onFalse(
              new InstantCommand(() -> {
                intake.stopIntake();
              }, intake));
    }

    //   Intake Default (operator triggers) 
    intake.setDefaultCommand(
      new RunCommand(() -> {
        intake.moveAtSpeed(operator.getRightTriggerAxis() - operator.getLeftTriggerAxis());
      }, intake)
    );

    { // Shoot
      driverLB
          .onTrue(
              new RunCommand(() -> {
                shooter.shoot();
                operator.setRumble(RumbleType.kBothRumble, 0.5);
              }, shooter))
          .onFalse(
              new InstantCommand(() -> {
                shooter.stop();
                operator.setRumble(RumbleType.kBothRumble, 0);
              }, shooter));
    }

    // Toggle Driving Mode
    driverStart.onTrue(toggledriveMode);

    // Reset Encoder
    driverBack.onTrue(resetdegree);

    // DriveTrain Default (driver sticks)
    driveTrain.setDefaultCommand(
        new RunCommand(() -> {
          driveTrain.drive(
              driver.getRawAxis(DriverConstants.leftY) * DriverConstants.driveMult,
              driver.getRawAxis(DriverConstants.leftX) * DriverConstants.driveMult,
              driver.getRawAxis(DriverConstants.rightX) * DriverConstants.driveMult);
        }, driveTrain));

    operatorStart.onTrue(toggleClimberLimit);

    // Climbers Default (operator sticks)
    climbers.setDefaultCommand(
        new RunCommand(() -> {
          climbers.runClimbers(operator.getLeftY(), operator.getRightY());
        }, climbers));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Test and choose autos
    return Autos.testAutoDrive(driveTrain, intake, shooter);
  }
}
