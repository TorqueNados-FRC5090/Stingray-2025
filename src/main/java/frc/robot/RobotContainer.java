package frc.robot;

import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;
import static frc.robot.Constants.ControllerPorts.OPERATOR_PORT;
import static frc.robot.Constants.DriveConstants.MAX_ROTATION_SPEED;
import static frc.robot.Constants.DriveConstants.MAX_TRANSLATION_SPEED;
import static frc.robot.Constants.DriveConstants.ROTATION_DEADBAND;
import static frc.robot.Constants.DriveConstants.TRANSLATION_DEADBAND;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.LEDControl;
import frc.robot.subsystems.CTRESwerveDrivetrain;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;

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
    public final Elevator elevator = new Elevator();
    public final Candle candleLEDS = new Candle();

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
        elevator.setDefaultCommand(elevator.elevateToPosition(ElevatorPosition.ZERO));
        candleLEDS.setDefaultCommand(new LEDControl(candleLEDS, this));
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
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.rightTrigger().whileTrue(shooter.shoot(.5));
        driverController.leftTrigger().whileTrue(shooter.shoot(-.15));
        
        driverController.pov(270).onTrue(climber.climbToPosition(ClimberPosition.CLIMB));
        driverController.pov(0).onTrue(climber.climbToPosition(ClimberPosition.ZERO));
        driverController.pov(90).onTrue(climber.climbToPosition(ClimberPosition.PREPARE));
    }
    
    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        operatorController.y().whileTrue(elevator.elevateToPosition(ElevatorPosition.L2));
        operatorController.x().whileTrue(elevator.elevateToPosition(ElevatorPosition.L3));
        operatorController.rightBumper().whileTrue(elevator.elevateToPosition(ElevatorPosition.L4));
        operatorController.a().whileTrue(elevator.elevateToPosition(ElevatorPosition.ZERO));
        operatorController.b().whileTrue(elevator.elevateToPosition(ElevatorPosition.TROUGH));
        
        operatorController.start().and(operatorController.back()).whileTrue(funnel.funnelDrop());
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
