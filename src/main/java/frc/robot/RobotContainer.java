// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.MoveIntakeInside;
import frc.robot.commands.MoveIntakeToAmp;
import frc.robot.commands.MoveIntakeToFloor;
import frc.robot.commands.ResetDegree;
import frc.robot.commands.ToggleDrivingMode;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  public final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();

  private final CommandJoystick driver = new CommandJoystick(DriverConstants.controllerPort);

  // Commands
  private final ToggleDrivingMode toggledriveMode = new ToggleDrivingMode(driveTrain);
  private final ResetDegree resetdegree = new ResetDegree(driveTrain);

  private final MoveIntakeToFloor moveIntakeToFloor = new MoveIntakeToFloor(intake);
  private final MoveIntakeInside moveIntakeInside = new MoveIntakeInside(intake);
  private final MoveIntakeToAmp moveIntakeToAmp = new MoveIntakeToAmp(intake);

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
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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

    driver.button(1).onTrue(moveIntakeToFloor);
    driver.button(2).onTrue(moveIntakeInside);
    driver.button(4).onTrue(moveIntakeToAmp);

    driver.button(DriverConstants.start).onTrue(toggledriveMode);

    driver.button(DriverConstants.back).onTrue(resetdegree);

    driveTrain.setDefaultCommand(
        new RunCommand(() -> {
          driveTrain.drive(
              driver.getRawAxis(DriverConstants.leftY) * DriverConstants.driveMult,
              driver.getRawAxis(DriverConstants.leftX) * DriverConstants.driveMult,
              driver.getRawAxis(DriverConstants.rightX) * DriverConstants.driveMult);
        }, driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
