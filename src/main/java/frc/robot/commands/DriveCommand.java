package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CTRESwerveDrivetrain;

public class DriveCommand extends Command {
    private CTRESwerveDrivetrain drivetrain;
    private DoubleSupplier inputX;
    private DoubleSupplier inputY;
    private DoubleSupplier inputR;
    private DoubleSupplier inputScalar;
    private BooleanSupplier robotCentric;

    private final SlewRateLimiter slewX = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TRANSLATION_SLEW);
    private final SlewRateLimiter slewR = new SlewRateLimiter(ROTATION_SLEW);


    /** Constructs a command for driving the robot with joysticks.
     * @param drivetrain The robot's drivetrain subsystem
     * @param inputX The side-to-side robot control [-1, 1] (positive right)
     * @param inputY The forward-backward robot control [-1, 1] (positive forward)
     * @param inputR The rotational robot control [-1, 1] (positive clockwise)
     * @param inputScalar A number to multiply to multiply the inputs by. Useful for conditional slowing
     */
    public DriveCommand(
        CTRESwerveDrivetrain drivetrain, 
        DoubleSupplier inputX, 
        DoubleSupplier inputY, 
        DoubleSupplier inputR, 
        DoubleSupplier inputScalar, 
        BooleanSupplier robotCentric
    ) {
        this.drivetrain = drivetrain;
        this.inputX = inputX;
        this.inputY = inputY;
        this.inputR = inputR;
        this.inputScalar = inputScalar;
        this.robotCentric = robotCentric;

        addRequirements(drivetrain);
    }


    @Override
    public void execute() {
        double x = inputX.getAsDouble();
        double y = inputY.getAsDouble();
        double r = inputR.getAsDouble();
        double scalar = inputScalar.getAsDouble();
        boolean isRobotCentric = robotCentric.getAsBoolean();

        /* First we'll apply a deadband to the inputs to prevent excessively small
         * inputs from being sent to the drivetrain. This can prevent accidental control, 
         * as well as ignoring inputs too small to be useful. This comes at the cost of 
         * some responsiveness from the robot. */
        x = MathUtil.applyDeadband(x, TRANSLATION_DEADBAND);
        y = MathUtil.applyDeadband(y, TRANSLATION_DEADBAND);
        r = MathUtil.applyDeadband(r, ROTATION_DEADBAND);

        /* Linear control can feel jerky or unnatural to a driver, so next, rather
         * than sending the inputs straight to the drivetrain (output = input * scalar)
         * we want to remodel the input as (output = input^2 * signum(input) * scalar)
         * Doing so creates an exponential control curve, giving the driver more precise
         * control at slow speeds, but less precise control at higher speeds. */
        x = x * x * Math.signum(x) * scalar;
        y = y * y * Math.signum(y) * scalar;
        r = r * r * Math.signum(r) * scalar;

        /* We'll also apply a slew limiter to limit the input's rate of change and smooth out the
         * input curve. This way, sudden or jerky inputs taken from the driver will be applied
         * to the robot more smoothly over time, at the cost of some reactivity and explosiveness. */
        x = slewX.calculate(x);
        y = slewY.calculate(y);
        r = slewR.calculate(r);

        /* The output should be limited to [-1, 1] since you can't output more than 100% speed or less than -100% */
        x = MathUtil.clamp(x, -1, 1);
        y = MathUtil.clamp(y, -1, 1);
        r = MathUtil.clamp(r, -1, 1);

        /* Currently, we have the control output in the joystick's terms of [-1, 1]. However, the drivetrain 
         * wants the output in terms of velocity, so we need to multiply the output by the max speed of the robot,
         * effectively mapping the control output to the robot's range of possible speeds. */
        x = x * MAX_TRANSLATION_SPEED;
        y = y * MAX_TRANSLATION_SPEED;
        r = r * MAX_ROTATION_SPEED;

        /* The final step is to construct a SwerveRequest using the computed speeds and send it to the drivetrain.
         * However, the drivetrain also expects speeds using a different coordinate system, so we have to 
         * map the output space to the drivetrain space like so:
         * Drivetrain x = Output -y
         * Drivetrain y = Output -x
         * Drivetrain r = Output -r
         */
        if (isRobotCentric) { // Send the request in field centric mode
            SwerveRequest.RobotCentric outputRequest = new SwerveRequest.RobotCentric();
            outputRequest
                .withVelocityX(-y)
                .withVelocityY(-x)
                .withRotationalRate(-r)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.Position);
            drivetrain.setControl(outputRequest);
        }
        else { // Send the request in robot centric mode
            SwerveRequest.FieldCentric outputRequest = new SwerveRequest.FieldCentric();
            outputRequest
                .withVelocityX(-y)
                .withVelocityY(-x)
                .withRotationalRate(-r)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.Position);
            drivetrain.setControl(outputRequest);
        }
    }
}
