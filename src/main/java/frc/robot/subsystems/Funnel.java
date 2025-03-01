package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.INTAKE_SERVO_PORT;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.ServoManager;

public class Funnel extends SubsystemBase {

    ServoChannel intakeServo;
    boolean dropped;

    public Funnel(){
        intakeServo = ServoManager.getInstance().getServoInPort(INTAKE_SERVO_PORT);
        dropped = false;
    }

    public Command funnelDrop(){
        return this.runOnce(() -> unlatch());
    }

    public boolean hasBeenDropped() {
        return dropped;
    }

    public void unlatch(){
        intakeServo.setPulseWidth(2500);
        dropped = true;
    }
     
    public void zero(){
        intakeServo.setPulseWidth(1500);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Servo", intakeServo.getPulseWidth());
    }
}
