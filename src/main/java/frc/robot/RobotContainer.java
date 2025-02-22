package frc.robot;

import static frc.robot.Constants.ControllerPorts.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutonContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CTRESwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {
    // CTRE drivetrain control functions
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TRANSLATION_DEADBAND).withRotationalDeadband(ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);
    // Subsystems
    public final LimeLight frontLimelight = new LimeLight("limelight-ultron");
    public final CTRESwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(frontLimelight);

    private final Funnel funnel = new Funnel();
    public final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Elevator elevator = new Elevator();

    // Misc objects
    private final AutonContainer auton = new AutonContainer(this);
    private final SendableChooser<Command> autonChooser = auton.buildAutonChooser();
    private final Telemetry logger = new Telemetry(MAX_TRANSLATION_SPEED);

    /** Constructs a RobotContainer */
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        drivetrain.registerTelemetry(logger::telemeterize);

        setDriverControls();
        setOperatorControls();
        setDefaultActions();
    }

    /** @return Whether the robot is on the red alliance or not */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    private void setDefaultActions() {
        shooter.setDefaultCommand(new AutoIntake(shooter));
        elevator.setDefaultCommand(elevator.elevateToPosition(ElevatorPosition.L2));
    }

    /** Configures a set of control bindings for the robot's driver */
    private void setDriverControls() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MAX_TRANSLATION_SPEED) 
                    .withVelocityY(-driverController.getLeftX() * MAX_TRANSLATION_SPEED)
                    .withRotationalRate(-driverController.getRightX() * MAX_ROTATION_SPEED) 
            )
        );

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() ->   drivetrain.seedFieldCentric()));
        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    }
    
    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        operatorController.rightTrigger().whileTrue(shooter.shoot(.5));
        operatorController.leftTrigger().whileTrue(shooter.shoot(-.5));
        
        operatorController.leftBumper().whileTrue(elevator.elevateToPosition(ElevatorPosition.L2));
        operatorController.rightBumper().whileTrue(elevator.elevateToPosition(ElevatorPosition.L3));
        
        operatorController.start().and(operatorController.back()).whileTrue(funnel.funnelDrop());
        operatorController.x().onTrue(climber.climbToPosition(ClimberPosition.CLIMB));
        operatorController.y().onTrue(climber.climbToPosition(ClimberPosition.ZERO));
        operatorController.b().onTrue(climber.climbToPosition(ClimberPosition.PREPARE));
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
