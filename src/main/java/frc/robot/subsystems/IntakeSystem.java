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

public class IntakeSystem extends SubsystemBase {
    private RelativeEncoder intakeEnco;
    private double kp = 2;
    private SparkPIDController m_PidController;
    private CANSparkMax motor;

    public IntakeSystem() {
        motor = new CANSparkMax(26, CANSparkLowLevel.MotorType.kBrushless);
        motor.setInverted(true);
        intakeEnco = motor.getEncoder();
        m_PidController = motor.getPIDController();
        intakeEnco.setPositionConversionFactor(1 / 80);
        intakeEnco.setPosition(0);
        m_PidController.setOutputRange(-2, 2);
        m_PidController.setP(0.1);
        m_PidController.setD(0.01);
        m_PidController.setI(0);
        m_PidController.setFF(0);
    }

    public void setPivotAngle(double angle) {
        m_PidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void stopMotor() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        System.out.println(intakeEnco != null ? intakeEnco.getPosition():"not initated yet");
    }

}
