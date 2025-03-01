package frc.robot.commands;

import static frc.robot.Constants.PathPlannerConfigs.PP_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.CTRESwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer {
    private CTRESwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Elevator elevator;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        this.drivetrain = robot.drivetrain;
        this.shooter = robot.shooter;
        this.elevator = robot.elevator;
        registerNamedCommands();

        // Attempt to load the pathplanner config from GUI
        // Fallback onto the config in Constants because it's better than crashing
        RobotConfig config = PP_CONFIG;
        try { config = RobotConfig.fromGUISettings(); }
        catch (Exception e) { e.printStackTrace(); }

        AutoBuilder.configure(
            drivetrain::getPose, 
            drivetrain::resetPose,
            drivetrain::getChassisSpeeds,
            (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> robot.onRedAlliance(),
            drivetrain
        );
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ElevatorToL4", new SetElevatorTarget(elevator, ElevatorPosition.L4));
        NamedCommands.registerCommand("ElevatorToL2", new SetElevatorTarget(elevator, ElevatorPosition.L2));
        NamedCommands.registerCommand("ElevatorToZero", new SetElevatorTarget(elevator, ElevatorPosition.ZERO));
        NamedCommands.registerCommand("Shoot", shooter.shoot(.5).withTimeout(.5));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        chooser.addOption("Right Single", AutoBuilder.buildAuto("Right Single"));
        chooser.addOption("Left Single", AutoBuilder.buildAuto("Left Single"));
        chooser.addOption("Center Single", AutoBuilder.buildAuto("Center Single"));
        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return new WaitCommand(0);
    }
}