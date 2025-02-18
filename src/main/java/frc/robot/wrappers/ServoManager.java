package frc.robot.wrappers;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

public class ServoManager {
    ServoHub servoHub;
    public ServoManager(int hubID){
        servoHub = new ServoHub(hubID);
    }
    
    public ServoChannel getServoInPort(ServoChannel.ChannelId channelID){
        return servoHub.getServoChannel(channelID);
    }
}
