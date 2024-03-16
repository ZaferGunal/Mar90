package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterSystem extends SubsystemBase {

    private double kP = 0.3;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kFF = 0.0;

    private SparkPIDController m_PidControllerRight;
    private SparkPIDController m_PidControllerLeft;
     CANSparkMax motorLeftShoot = new CANSparkMax(20,MotorType.kBrushless);
  CANSparkMax motorRightShoot = new CANSparkMax(21,MotorType.kBrushless);

  


   public ShooterSystem(){
   m_PidControllerLeft = motorLeftShoot.getPIDController();
   m_PidControllerLeft.setP(kP);
   m_PidControllerLeft.setI(kI);
   m_PidControllerLeft.setD(kD);
   m_PidControllerLeft.setFF(kFF);
    m_PidControllerRight = motorRightShoot.getPIDController();
   m_PidControllerRight.setP(kP);
   m_PidControllerRight.setI(kI);
   m_PidControllerRight.setD(kD);
   m_PidControllerRight.setFF(kFF);
   

   motorRightShoot.setInverted(true);
   }
    
  
public void shoot(double speed){
  //  m_PidControllerLeft.setReference(speed, CANSparkMax.ControlType.kVelocity);
   // m_PidControllerRight.setReference(speed,CANSparkMax.ControlType.kVelocity);
   motorLeftShoot.setVoltage(2.75);
   motorRightShoot.setVoltage(10);

}



public void stopMotors(){
    motorLeftShoot.set(0);
    motorRightShoot.set(0);
}
}