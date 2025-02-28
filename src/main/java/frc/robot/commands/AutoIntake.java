package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command {
    private Shooter shooter;
    private IntakeState state;
    private double fastSpeed = .3;
    private double slowSpeed = .13;

    private boolean entry;
    private boolean exit;

    /** A list of the states the intake can be in */
    public enum IntakeState {
        FAST,
        SLOW,
        DONE;
    }


    /** Constructs an AutoIntake Command
     * This command attempts to index pieces as they come into the intake.
     * It is meant to be run as a default command for the system, and cancelled
     * by another command requiring the subsystem when pieces are shot.
     * This is important because the cancellation should trigger a rerun of the
     * initialize function.
     * 
     * @param shooter The intake/shooter system used to index pieces.
     */
    public AutoIntake(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
        if(shooter.isExitSensorBlocked())
            state = IntakeState.DONE;
        else
            state = IntakeState.FAST;
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        entry = shooter.isEntrySensorBlocked();
        exit = shooter.isExitSensorBlocked();

        if (state == IntakeState.FAST) {
            if (entry) {
                shooter.spin(slowSpeed);
                state = IntakeState.SLOW;
            }
            else if (exit)
                state = IntakeState.DONE;
            else
                shooter.spin(fastSpeed);
        }

        else if (state == IntakeState.SLOW) {
            if (!entry) {
                shooter.stop();
                state = IntakeState.DONE;
            }
            else
                shooter.spin(slowSpeed);
        }

        else if (state == IntakeState.DONE) {
            if (!entry && !exit)
                state = IntakeState.FAST;
            else
                shooter.stop();
        }
    }
}
