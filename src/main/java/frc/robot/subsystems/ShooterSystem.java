package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;


public class ShooterSystem extends SubsystemBase{

    CANSparkMax left = new CANSparkMax(24,CANSparkLowLevel.MotorType.kBrushless);
    

    
    public void turnForward(double speed_){
        left.set(speed_);
    }

    public void turnBackward(Double speed__){
        left.set(speed__);
    }

    public void stopMotors(){
        left.set(0);
        
    }
}
