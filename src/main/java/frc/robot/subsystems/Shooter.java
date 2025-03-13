package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.SHOOTER_ENTRY_SENSOR_ID;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_EXIT_SENSOR_ID;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_MOTOR_ID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    TalonFX leadMotor;
    Canandcolor entrySensor;
    Canandcolor exitSensor;  
    
    public Shooter(){
        leadMotor = new TalonFX(SHOOTER_MOTOR_ID);

        // Configure the motors
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leadMotor.getConfigurator().apply(leaderConfig);
        
        
        // Initialize and configure the sensors
        entrySensor = new Canandcolor(SHOOTER_ENTRY_SENSOR_ID);
        entrySensor.resetFactoryDefaults();
        exitSensor = new Canandcolor(SHOOTER_EXIT_SENSOR_ID);
        exitSensor.resetFactoryDefaults();

        CanandcolorSettings settings = new CanandcolorSettings();
        settings.setLampLEDBrightness(0);
        entrySensor.setSettings(settings);
        exitSensor.setSettings(settings);
    }


    /** @return A command that drives the shooter at a given speed and stops when the command is cancelled */
    public Command shoot(double speed) {
        return startEnd(
            () -> spin(speed),
            () -> stop()
        );
    }


    public void spin(double speed) { 
        leadMotor.set(speed); 
    }
    public void stop() { 
        leadMotor.stopMotor(); 
    }

    public void setEncoder(double position) { leadMotor.setPosition(position); }
    public double getPosition() { return leadMotor.getPosition().getValueAsDouble(); }
    public double getVelocity() { return leadMotor.getVelocity().getValueAsDouble(); }

    /** @return Whether there is a piece in front of the sensor at the end of the shooter */
    public boolean isEntrySensorBlocked() { return entrySensor.getProximity() <= .05; }
    /** @return Whether there is a piece in front of the sensor at the robot's intake */
    public boolean isExitSensorBlocked() { return exitSensor.getProximity() <= .05; }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getVelocity());
        SmartDashboard.putNumber("Shooter Position", getPosition());
        SmartDashboard.putBoolean("Shooter Entry Sensor", isEntrySensorBlocked());
        SmartDashboard.putBoolean("Shooter Exit Sensor", isExitSensorBlocked());
    }
}
