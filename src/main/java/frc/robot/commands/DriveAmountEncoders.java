package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveAmountEncoders extends Command {
    private DriveTrain driveTrain;
    private double forward, strafe, turn;

    private boolean encoderReset = false;

  /** Creates a new DriveAmount. */
  public DriveAmountEncoders(double fowardDist, double strafeDist, double turnDist, DriveTrain dt) {
    driveTrain = dt;
    forward = fowardDist;
    strafe = strafeDist;
    turn = turnDist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!encoderReset) {
            encoderReset = true;
            driveTrain.resetEncoders();
        }

        driveTrain.drive(forward, strafe, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
