package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_ENTRY_SENSOR_ID;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_EXIT_SENSOR_ID;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.SHOOTER_RIGHT_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

public class Shooter extends SubsystemBase {
    SparkFlex leadMotor;
    SparkFlex followMotor;
    GenericPID shooterPID;
    boolean active = false;

    Canandcolor entrySensor;
    Canandcolor exitSensor;  
    
    public Shooter(){
        leadMotor = new SparkFlex(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
        followMotor = new SparkFlex(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        shooterPID = new GenericPID(leadMotor, ControlType.kPosition, .1);
        
        // Configure the motors
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followConfig = new SparkMaxConfig();
        leaderConfig.encoder.positionConversionFactor(SHOOTER_RATIO);
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followConfig.follow(leadMotor, true);
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
        
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
        return runEnd(
            () -> spin(speed),
            () -> stop()
        );
    }


    public void indexPiece() {
        if(!active) {
            shooterPID.activate(IDEAL_CORAL_POSITION);
            active = true;
        }
    }

    public void spin(double speed) { 
        leadMotor.set(speed); 
        if(active) {
            shooterPID.pause(); 
            active = false;
        }
    }
    public void stop() { 
        leadMotor.stopMotor(); 
        if(active) {
            shooterPID.pause();
            active = false;
        }
    }

    public void setEncoder(double position) { leadMotor.getEncoder().setPosition(position); }
    public double getPosition() { return leadMotor.getEncoder().getPosition(); }
    public double getVelocity() { return leadMotor.getEncoder().getVelocity(); }

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
