package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import frc.robot.SystemConstants.IntakeConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;


public class IntakeSystem  extends SubsystemBase




{



private CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.rollerMotorId,CANSparkLowLevel.MotorType.kBrushless);
    

private RelativeEncoder pivotEncoder ;
private SparkPIDController pivotPIDController;
private CANSparkMax pivotMotor;

public IntakeSystem(){
    pivotMotor = new CANSparkMax(IntakeConstants.pivotMotorId,CANSparkLowLevel.MotorType.kBrushless);
    pivotMotor.setInverted(IntakeConstants.pivotMotorInverted);


    pivotPIDController = pivotMotor.getPIDController();
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPositionConversionFactor(1/80);
    pivotEncoder.setPosition(0);

    pivotPIDController.setP(IntakeConstants.pivotPID_P);
    pivotPIDController.setI(IntakeConstants.pivotPID_I);
    pivotPIDController.setD(IntakeConstants.pivotPID_D);
    pivotPIDController.setFF(IntakeConstants.pivotPID_FF);

    pivotPIDController.setOutputRange(-1,1);

}



// pivot methods
public void setPivot2floor(){
    pivotPIDController.setReference(IntakeConstants.floorPosition,CANSparkBase.ControlType.kPosition);

}

public void setPivot2mid(){
     pivotPIDController.setReference(IntakeConstants.midPosition,CANSparkBase.ControlType.kPosition);

}

public void setPivot2feed(){
     pivotPIDController.setReference(IntakeConstants.feedPosition,CANSparkBase.ControlType.kPosition);

}



// roller methods

public void rollerCapture(){
    rollerMotor.setVoltage(IntakeConstants.rollerCaptureVoltage);
}

public void rollerPush(){
    rollerMotor.setVoltage(IntakeConstants.rollerPushVoltage);
}

public void rollerHold(){
  rollerMotor.setVoltage(IntakeConstants.rollerHoldVoltage);
}

public void rollerPush4amp(){
  rollerMotor.setVoltage(IntakeConstants.push4AmpVoltage);
}



// stop methods

public void stopPivotMotor(){
pivotMotor.set(0);
    
} 
public void stopRollerMotor(){
 rollerMotor.set(0);
}
public void stopAllIntakeMotors(){
    rollerMotor.set(0);
    pivotMotor.set(0);
}









@Override
public void periodic() {
  //  System.out.println("pivot angle:   " + pivotEncoder.getPosition());
  

}
    
}
