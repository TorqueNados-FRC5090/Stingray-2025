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

import frc.robot.wrappers.GenericPID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    SparkFlex leadMotor;
    SparkFlex followMotor;
    Canandcolor entrySensor;
    Canandcolor exitSensor;  
    CanandcolorSettings settings;
    GenericPID ShooterPID;

    public Shooter(){
        leadMotor = new SparkFlex(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
        followMotor = new SparkFlex(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
        ShooterPID = new GenericPID(leadMotor, ControlType.kPosition, P_GAIN);
       
        // Configure the motors
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followConfig = new SparkMaxConfig();
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followConfig.follow(leadMotor, true);
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);

        // Initialize and configure the sensors
        entrySensor = new Canandcolor(SHOOTER_ENTRY_SENSOR_ID);
        entrySensor.resetFactoryDefaults();
        exitSensor = new Canandcolor(SHOOTER_EXIT_SENSOR_ID);
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

    /** @return A command that moves the Shooter to a position */
    public Command shootToPosition(double setpoint) {
        return runOnce(() -> driveShooterToPosition(setpoint));
    }

        /** Drives the Shooter to a position using the PID controller on its controller */
    public void driveShooterToPosition(double setpoint) { ShooterPID.activate(setpoint); }
    
    public double getshooterposition(){
        return ShooterPID.getMeasurement();
    }

    public boolean shooteratposition(){
        return ShooterPID.atSetpoint(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", leadMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Shooter Entry Sensor", isEntrySensorBlocked());
        SmartDashboard.putBoolean("Shooter Exit Sensor", isExitSensorBlocked());
        SmartDashboard.putBoolean("Shooter At Setpoint", shooteratposition());
        SmartDashboard.putNumber("ShooterPosition", leadMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ShooterSetpoint", ShooterPID.getSetpoint());
    }
}
