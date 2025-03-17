package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

    PWM servoPort;
    boolean dropped;

    public Funnel(){
        servoPort = new PWM(0);
        servoPort.setPosition(0);
        dropped = false;
    }

    public Command funnelDrop(){
        return this.startEnd(
            () -> unlatch(),
            () -> zero()
        );
    }

    public boolean hasBeenDropped() {
        return dropped;
    }

    public void unlatch(){
        servoPort.setPosition(1);
        dropped = true;
    }
     
    public void zero(){
        servoPort.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Servo", servoPort.getPosition());
    }
}
