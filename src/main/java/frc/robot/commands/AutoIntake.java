package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SENSOR_SEPARATION;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command {
    private Shooter shooter;
    private IntakeState state;
    private double[] measurements;
    private final double coralLength = 11.875;

    private boolean entry;
    private boolean exit;
    private boolean lastEntry;
    private boolean lastExit;

    public enum IntakeState {
        Waiting,
        Localizing,
        Indexing;
    }
    

    public AutoIntake(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        state = IntakeState.Waiting;
        measurements = new double[3];
        entry = false;
        exit = false;
        lastEntry = false;
        lastExit = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        entry = shooter.isEntrySensorBlocked();
        exit = shooter.isExitSensorBlocked();

        transition();

        lastEntry = entry;
        lastExit = exit;
    }

    private void transition() {
        switch (state) {
            case Waiting:
                /* When a piece enters the intake, index it to desired position */
                if (!entry && lastEntry)
                    state = IntakeState.Indexing;
                /* If there is already a piece in the intake, localize it */
                else if (entry)
                    state = IntakeState.Localizing;
                break;

            case Localizing:
                /* When the state of either sensor changes, we've found the piece and can index it */
                if (entry != lastEntry || exit != lastExit)
                    state = IntakeState.Indexing;
                break;

            case Indexing: break; // Never leave the indexing state

            // In the case of an invalid state, set to waiting
            default:
                state = IntakeState.Waiting;
        }
    }
}
