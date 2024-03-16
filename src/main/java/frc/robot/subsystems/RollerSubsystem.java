package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

public class RollerSubsystem extends SubsystemBase {
    private CANSparkMax motorRoller;

    public RollerSubsystem() {
        motorRoller = new CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless);
    }

    
    public void setRollerSpeed(double voltage){
        motorRoller.setVoltage(voltage);
    }


       public void stopRoller() {
        
        motorRoller.set(0);
    }



    @Override
    public void periodic() {
    }

}
