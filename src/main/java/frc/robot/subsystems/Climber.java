// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.ElevatorConstants;

//CTRE Imports Motor Imports
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {


  private double position;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab tab = Shuffleboard.getTab("Climber");
  private GenericEntry climberEncoder = tab.add("Climber Encoder", 0).getEntry();
private GenericEntry climberVoltage =
      tab.add("Climber Voltage", 0)
         .getEntry();
  /** Creates a new ElevatorSubsystem. */


  private final TalonFX climber = new TalonFX(51); //LEFT is LEADER
 /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  //private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  //private final VelocityVoltage m_velocityTorque = new VelocityVoltage(0).withSlot(1);


  public Climber() {

    position = 0;

  
      TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

          /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot2.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot2.kP = 5; // An error of 1 rotation per second results in 5 A output
    configs.Slot2.kI = 0; // No output for integrated error
    configs.Slot2.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));  
      climber.getConfigurator().apply(configs);
      climber.setNeutralMode(NeutralModeValue.Brake);
      climber.setPosition(position);

  }

public void setHoldPosition(double holdposition) {
  position = holdposition;

}

public void setVelocity(double speed)
{
  climber.setControl(m_velocityVoltage.withVelocity(speed));
}

// public boolean CheckPositionAmp()
// {
//  return MathUtil.isNear(ElevatorConstants.eAmp,m_leftElevator.getEncoder().getPosition(), 1);
// }
// public boolean CheckPositionHome()
// {
//  return MathUtil.isNear(ElevatorConstants.eHomePos,m_leftElevator.getEncoder().getPosition(), 1);
// }

private void setPosition(double setPoint)
{
  //m_positionTorque.withPosition(setPoint);
}

public void homePosition()
{
   // l_pidController.setReference(ElevatorConstants.eHomePos, CANSparkMax.ControlType.kPosition);
   // position = ElevatorConstants.eHomePos;
}

public double getEncoder()
{
  return climber.getPosition().getValueAsDouble();
}

public Command slowUp()
{
  return run(() -> this.setVelocity(20));
}


public Command slowDown()
{
  return run(() -> this.setVelocity(-20));
}


public Command stop()
{
  return run(() -> this.setVelocity(0));
}


public Command withPosition(double setPoint)
{
  return runOnce(() -> this.setPosition(setPoint));
}

public Command climbPosition()
{
  return run(() -> this.setPosition(0));
}

public Command setupPosition()
{
  return run(() -> this.setPosition(0));
}


public Command setHomePosition()
{
  return run(() -> this.setPosition(0)/* .until(()-> this.CheckPositionHome())*/); // need to find
}


// public boolean LimitChecks()
// {
// return ((l_encoder.getPosition() > -1.5 && m_leftElevator.getAppliedOutput() > 0) || (l_encoder.getPosition() < ElevatorConstants.ELEVATORMAX && m_leftElevator.getAppliedOutput() < 0));
// }

public void end() {

}



@Override
public void periodic() {
  // This method will be called once per scheduler run

// elevatorEncoder.setDouble(l_encoder.getPosition());
//SmartDashboard.putBoolean("limit checks", LimitChecks());
//elevatorVoltage.setDouble(m_leftElevator.getAppliedOutput());
SmartDashboard.putNumber("Climber Encoder", climber.getPosition().getValueAsDouble());
//SmartDashboard.putNumber("Elevator Appied Voltage", m_leftElevator.getAppliedOutput() );
}
}