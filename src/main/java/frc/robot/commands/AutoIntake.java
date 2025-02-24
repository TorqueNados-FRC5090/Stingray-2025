package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command{
    Shooter shooter;
    double counter = 1;
  
    public AutoIntake(Shooter shooter){
       this.shooter = shooter;
       addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.isEntrySensorBlocked() && counter == 1){
            //shooter.shoot(.25);
            shooter.driveShooterToPosition(ShooterPosition.Intake1.getcurrentpos()+shooter.getshooterposition());
            counter = counter + 1;
        }
        else if (shooter.isEntrySensorBlocked() && counter == 2){
            shooter.driveShooterToPosition(ShooterPosition.Slow.getcurrentpos()+shooter.getshooterposition());
            if (shooter.isExitSensorBlocked()){
                counter = 3;
            }

        }
        else if (shooter.isExitSensorBlocked()){
            shooter.spin(0);
        }
        else {
            shooter.spin(.15);
            counter = 1;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}
