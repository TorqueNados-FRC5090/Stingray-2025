package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import static frc.robot.Constants.ShooterConstants.*;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkFlex leadMotor;
    SparkFlex followMotor;
    Canandcolor entrySensor;
    Canandcolor exitSensor;  
    CanandcolorSettings settings;

    public Shooter(){
        leadMotor = new SparkFlex(LEFT_MOTOR_ID, MotorType.kBrushless);
        followMotor = new SparkFlex(RIGHT_MOTOR_ID, MotorType.kBrushless);
       
        // Configure the motors
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotor, true);
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);

        // Initialize and configure the sensors
        entrySensor = new Canandcolor(ENTRY_SENSOR_ID);
        entrySensor.resetFactoryDefaults();
        exitSensor = new Canandcolor(EXIT_SENSOR_ID);
        exitSensor.resetFactoryDefaults();
        settings = new CanandcolorSettings();

        settings.setLampLEDBrightness(0);
        entrySensor.setSettings(settings);
        exitSensor.setSettings(settings);
    }


    /** @return A command that drives the shooter at a given speed and stops when the command is cancelled */
    public Command shoot(double speed) {
        return runEnd(
            () -> spin(speed), 
            () -> stop()
        );
    }


    public void spin(double speed) { leadMotor.set(speed); }
    public void stop() { leadMotor.stopMotor(); }

    /** @return Whether there is a piece in front of the sensor at the end of the shooter */
    public boolean isEntrySensorBlocked() { return entrySensor.getProximity() <= .05; }
    /** @return Whether there is a piece in front of the sensor at the robot's intake */
    public boolean isExitSensorBlocked() { return exitSensor.getProximity() <= .05; }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", leadMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter Entry Sensor", isEntrySensorBlocked());
        SmartDashboard.putBoolean("Shooter Exit Sensor", isExitSensorBlocked());
    }
}
