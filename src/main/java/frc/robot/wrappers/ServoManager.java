package frc.robot.wrappers;

import static frc.robot.Constants.ServoPorts.SERVO_HUB_CAN_ID;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

/** A singleton class that wraps the ServoHub class to make it statically accessible. */
public class ServoManager {
    private static ServoManager instance;
    private final ServoHub servoHub;

    /** Construct the ServoManager by initializing the ServoHub */
    private ServoManager() {
        servoHub = new ServoHub(SERVO_HUB_CAN_ID);
    }

    /** Returns the servo hub on the robot */
    public static ServoManager getInstance() {
        if (instance == null) {
            instance = new ServoManager();
        }
        return instance;
    }

    /** Returns the servo object that communicates with a given port 
     *  Note: The servo object will be enabled and powered automatically.
    */
    public ServoChannel getServoInPort(int portNumber) {
        ServoChannel servo = servoHub.getServoChannel(ServoChannel.ChannelId.fromInt(portNumber));
        servo.setEnabled(true);
        servo.setPowered(true);
        return servo;
    }
}
