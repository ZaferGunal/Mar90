package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;

public  class IntakeSystem extends SubsystemBase {

    public final double radius =  20 ;
    public final double gearRatio= 1 /80;
    public final double rotation2angle = Math.PI *gearRatio *radius * 2;
    CANSparkMax pivotMotor = new CANSparkMax(24,MotorType.kBrushless);


    public void setBackward(){
        pivotMotor.setVoltage(12);
    }
    
    public void setforward(){
        pivotMotor.set(-12);
    }

    public void stopMotor(){
   pivotMotor.set(0);
    }

        
    
 
}
