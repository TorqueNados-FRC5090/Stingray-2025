package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkFlex leadMotor;
    SparkFlex followMotor;
    // Create a new settings object
    CanandcolorSettings settings;
    // Creates a Canandcolor object referencing a Canandcolor with CAN ID 3
    Canandcolor canandcolorSlow;
    //Creates a Canandcolor object referencing a Canandcolor with CAN ID 0
    Canandcolor canandcolorStop;  

    public Shooter(int leadID, int followID, int slowID, int stopID){
        leadMotor = new SparkFlex(leadID, MotorType.kBrushless);
        followMotor = new SparkFlex(followID, MotorType.kBrushless);
       
       // configuration for follower motor
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadID, true);
        
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);

        canandcolorSlow = new Canandcolor(slowID);
        canandcolorSlow.resetFactoryDefaults();
        canandcolorStop = new Canandcolor(stopID);
        canandcolorStop.resetFactoryDefaults();

        settings = new CanandcolorSettings();
        settings.setLampLEDBrightness(0);

        canandcolorSlow.setSettings(settings);
        canandcolorStop.setSettings(settings);
    }

    public void shoot(double speed){
        leadMotor.set(speed);
    }

    public void stop(){
        leadMotor.set(0);
    }
    
    public boolean isSlowSensorBlocked() {
        return canandcolorSlow.getProximity() <= .05;
    }
    public boolean isStopSensorBlocked() {
        return canandcolorStop.getProximity() <= .05;
    }
}
