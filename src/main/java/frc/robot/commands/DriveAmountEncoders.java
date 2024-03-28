package frc.robot.commands;

import com.fasterxml.jackson.databind.deser.std.StringArrayDeserializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveAmountEncoders extends Command {
    private DriveTrain driveTrain;
    private double forwardDist, strafeDist, turnDist;
    private double forwardPower, strafePower, turnPower;
    private double frontLeftDist, frontRightDist, rearLeftDist, rearRightDist;
    private boolean encoderReset = false;

  /** Creates a new DriveAmount. */
  public DriveAmountEncoders(double forwardDist, double strafeDist, double turnDist, DriveTrain dt) {
    driveTrain = dt;
    this.forwardDist = forwardDist;
    this.strafeDist = strafeDist;
    this.turnDist = turnDist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        frontLeftDist = forwardDist + strafeDist + turnDist;
        rearLeftDist = forwardDist - strafeDist + turnDist;
        frontRightDist = forwardDist - strafeDist - turnDist;
        rearRightDist = forwardDist + strafeDist - turnDist;

        double maxComponent = Math.max(Math.max(Math.abs(strafeDist), Math.abs(forwardDist)), Math.abs(turnDist));

        forwardPower = forwardDist / maxComponent / 2;
        strafePower = strafeDist / maxComponent / 2;
        turnPower = turnDist / maxComponent / 2;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!encoderReset) {
            encoderReset = true;
            driveTrain.resetEncoders();
        }

        driveTrain.drive(forwardPower, strafePower, turnPower, false, false, false, false, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0, 0, 0, false, false, false, false, false);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(frontLeftDist, driveTrain.frontLeftEncoder.getPosition(), 3) &&
        MathUtil.isNear(frontRightDist, driveTrain.frontRightEncoder.getPosition(), 3) &&
        MathUtil.isNear(rearLeftDist, driveTrain.rearLeftEncoder.getPosition(), 3) &&
        MathUtil.isNear(rearRightDist, driveTrain.rearRightEncoder.getPosition(), 3);
    }

}
