// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystemOLD extends SubsystemBase { 
  private SparkMax m_climber;
  private SparkClosedLoopController c_pidController;

  private RelativeEncoder c_encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystemOLD() {
  m_climber = new SparkMax(ClimberConstants.climber, MotorType.kBrushless);

SparkMaxConfig config = new SparkMaxConfig();
     
     // PID coefficients
     kP = 0.03; 
     kI = 0;
     kD = 0; 
     kIz = 0; 
     kFF = 0.000015; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5700;


config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(kP, kI, kD, kFF, ClosedLoopSlot.kSlot0);

    m_climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  
     
  
  }
  private void resetEnc()
  {
    c_encoder.setPosition(0);

  }
  
  private void setVelocity(double setPoint)
  {
    m_climber.set(-setPoint);
   SmartDashboard.putNumber("Right Drive Encoder", c_encoder.getPosition());

  }
  
  private void setPosition(double setPoint)
  {
    c_pidController.setReference(setPoint, SparkMax.ControlType.kPosition);

  }
  
  public Command withVelocity(double setPoint)
  {
    return runOnce(() -> this.setVelocity(setPoint));
  }
  
  public Command slowUp()
  {
    return run(() -> this.setVelocity(.1));
  }
  
  public Command slowDown()
  {
    return run(() -> this.setVelocity(-.1));
  }
  
  public Command stop()
  {
    return run(() -> this.setVelocity(0));
  }
  
  public Command withPosition(double setPoint)
  {
    return run(() -> this.setPosition(setPoint));
  }
  
  public Command setHomePosition()
  {
    return run(() -> this.setPosition(ClimberConstants.cHomePos)); 
  }
  
  public Command setUpPosition()
  {
    return run(() -> this.setPosition(-ClimberConstants.cUpPose)); 
  }
  
  public Command ClimbedPosition()
  {
    return run(() -> this.setPosition(-ClimberConstants.cClimbPos)); 
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Climber1 Pivot Encoder", c_encoder.getPosition());
  }
  
  }

