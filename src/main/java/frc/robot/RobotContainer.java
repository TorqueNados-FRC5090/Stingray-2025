package frc.robot;

import static frc.robot.Constants.ControllerPorts.DRIVER_PORT;
import static frc.robot.Constants.ControllerPorts.OPERATOR_PORT;
import static frc.robot.Constants.DriveConstants.MAX_TRANSLATION_SPEED;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants.AlgaePosition;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SetElevatorTarget;
import frc.robot.subsystems.CTRESwerveDrivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AlgaeRemover;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_PORT);
    // Subsystems
    public final LimeLight frontLimelight = new LimeLight("limelight-ultron");
    public final CTRESwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(frontLimelight);

    public final Funnel funnel = new Funnel();
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    //public final Candle candleLEDS = new Candle();
    public final AlgaeRemover algaeRemover = new AlgaeRemover();

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
        //candleLEDS.setDefaultCommand(new LEDControl(candleLEDS, this));
    }

    /** Configures a set of control bindings for the robot's driver */
    private void setDriverControls() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, 
                () -> driverController.getLeftX(),
                () -> driverController.getLeftY(),
                () -> driverController.getRightX(),
                /* Left Bumper is used as the slow driving button.
                 * While held, the speed of the robot is multiplied by .8 
                 * The speed is also reduced if the elevator is raised by more than 20 inches. */
                () -> {
                    if (driverController.leftTrigger().getAsBoolean() || driverController.leftBumper().getAsBoolean()) 
                        return .6;
                    else if (elevator.getHeight() > 30)
                        return .4;
                    else
                        return 1;
                },
                /* Left Trigger is used as the robot centric button.
                 * While held, the robot will drive in robot centric mode. */
                () -> driverController.leftBumper().getAsBoolean()
            )
        );

        // Reset the field-centric heading on A press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // Put the robot in brake mode while X is held
        driverController.a().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

        driverController.rightTrigger().whileTrue(shooter.shoot(.2));
        
        driverController.pov(0).onTrue(climber.climbToPosition(ClimberPosition.ZERO));
        driverController.pov(90).onTrue(climber.climbToPosition(ClimberPosition.PREPARE));
    }
    
    /** Configures a set of control bindings for the robot's operator */
    private void setOperatorControls() {
        // When a button is pressed, start going to its position, return to zero when button is released
        operatorController.a().onTrue(new SetElevatorTarget(elevator, ElevatorPosition.TROUGH))
            .onFalse(new SetElevatorTarget(elevator, ElevatorPosition.ZERO));                
        operatorController.b().onTrue(new SetElevatorTarget(elevator, ElevatorPosition.L2))
            .onFalse(new SetElevatorTarget(elevator, ElevatorPosition.ZERO));
        operatorController.x().onTrue(new SetElevatorTarget(elevator, ElevatorPosition.L3))
            .onFalse(new SetElevatorTarget(elevator, ElevatorPosition.ZERO));
        operatorController.y().onTrue(new SetElevatorTarget(elevator, ElevatorPosition.L4))
            .onFalse(new SetElevatorTarget(elevator, ElevatorPosition.ZERO));

        operatorController.leftBumper().whileTrue(shooter.shoot(-.2));
        operatorController.rightBumper().whileTrue(
            algaeRemover.AlgaeRemoveArmOUt(AlgaePosition.OUT)
            .alongWith(shooter.shoot(-.3))
        );
        
        operatorController.start().and(operatorController.back()).whileTrue(funnel.funnelDrop());
        operatorController.pov(90).onTrue(
            climber.climbToPosition(ClimberPosition.CLIMB)
                .onlyIf(() -> funnel.hasBeenDropped())
        );
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
