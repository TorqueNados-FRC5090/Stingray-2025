package frc.robot.subsystems;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.ServoManager;

public class Funnel extends SubsystemBase {

    ServoChannel intakeServo;
    
    public Funnel(){
      ServoManager manager = new ServoManager(3);
       
    // Obtain a servo channel controller
    intakeServo = manager.getServoInPort(ChannelId.kChannelId3);
    intakeServo.setPowered(true);
    intakeServo.setEnabled(true);    
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
