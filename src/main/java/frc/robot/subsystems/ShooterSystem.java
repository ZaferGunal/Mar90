package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.SystemConstants.IntakeConstants;
import frc.robot.SystemConstants.ShooterConstants;
public class ShooterSystem extends SubsystemBase
{

    private CANSparkMax shootLeftMotor;
    private CANSparkMax shootRightMotor;
    public ShooterSystem(){
        shootLeftMotor = new CANSparkMax(ShooterConstants.shooterLeftMotorId,CANSparkLowLevel.MotorType.kBrushless);
         shootRightMotor = new CANSparkMax(ShooterConstants.shooterRightMotorId,CANSparkLowLevel.MotorType.kBrushless);

         shootLeftMotor.setInverted(ShooterConstants.shooterLeftMotorInverted);
         shootRightMotor.setInverted(ShooterConstants.shooterRightMotorInverted);

    }



    public void shoot2Spkr(){
        shootLeftMotor.setVoltage(ShooterConstants.speakerShootingVoltage);
        shootRightMotor.setVoltage(ShooterConstants.speakerShootingVoltage);
    }


    public void shoot2amp(){
        shootLeftMotor.setVoltage(ShooterConstants.ampShootingVoltage);
        shootRightMotor.setVoltage(ShooterConstants.ampShootingVoltage);
    }


    public void stopShooter(){
        shootLeftMotor.set(0);
        shootRightMotor.set(0);

    }

    



    
}
