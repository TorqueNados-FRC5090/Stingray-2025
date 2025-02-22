package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.INTAKE_SERVO_PORT;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.ServoManager;

public class Funnel extends SubsystemBase {

    ServoChannel intakeServo;

    public Funnel(){
        intakeServo = ServoManager.getInstance().getServoInPort(INTAKE_SERVO_PORT);
    }

    public Command funnelDrop(){
        return this.runEnd(
            () -> unlatch(),
            () -> zero()
        );
    }

    public void unlatch(){
        intakeServo.setPulseWidth(2500);
    }
     
    public void zero(){
        intakeServo.setPulseWidth(1500);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Servo", intakeServo.getPulseWidth());
    }
}
