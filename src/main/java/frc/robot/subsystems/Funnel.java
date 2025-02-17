package frc.robot.subsystems;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

    ServoChannel channel3;
    
    public Funnel(){
      
        // Initialize the servo hub
ServoHub servoHub = new ServoHub(3);
        // Obtain a servo channel controller
 channel3 = servoHub.getServoChannel(ChannelId.kChannelId3);


        
    }

    public void unlatch(){
        channel3.setPulseWidth(2500);
        
    }
     
    public void zero(){
        channel3.setPulseWidth(500);
    }
}
