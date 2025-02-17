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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {


  private double position;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
  private GenericEntry elevatorEncoder = tab.add("Elevator Encoder", 0).getEntry();
private GenericEntry elevatorVoltage =
      tab.add("Elevator Voltage", 0)
         .getEntry();
  /** Creates a new ElevatorSubsystem. */


  private final TalonFX elevatorLead = new TalonFX(31); //LEFT is LEADER
  private final TalonFX elevatorFollower = new TalonFX(32); //RIGHT is FOLLOWER
 /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);


  double home = 0;
  double L4 = 0;
  double L3 = 0;
  double L2 = 0;
  double feedposition = 0;
  double L1 = 0;

  public ElevatorSubsystem() {

    position = 0;

  
      TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
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
      elevatorLead.getConfigurator().apply(configs);
      elevatorFollower.setControl(new Follower(elevatorLead.getDeviceID(), true));
      elevatorLead.setNeutralMode(NeutralModeValue.Brake);
      elevatorFollower.setNeutralMode(NeutralModeValue.Brake);
      elevatorLead.setPosition(position);

  }

public void setHoldPosition(double holdposition) {
  position = holdposition;

}

public void setVelocity(double speed)
{
  elevatorFollower.setControl(m_velocityTorque.withVelocity(speed));
  position = elevatorFollower.getPosition().getValueAsDouble();
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
  position = setPoint;
}

public double getEncoder()
{
  return elevatorLead.getPosition().getValueAsDouble();
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
  return runOnce(() -> this.setPosition(setPoint));
}

public Command homePosition()
{
  return run(() -> this.setPosition(home));
}

public Command L4Position()
{
  return run(() -> this.setPosition(L4));
}

public Command L3Position()
{
  return run(() -> this.setPosition(L3));
}

public Command L2Position()
{
  return run(() -> this.setPosition(L2));
}

public Command L1Position()
{
  return run(() -> this.setPosition(L1));
}

public Command FeedPosition()
{
  return run(() -> this.setPosition(feedposition));
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

m_positionTorque.withPosition(position);

// elevatorEncoder.setDouble(l_encoder.getPosition());
//SmartDashboard.putBoolean("limit checks", LimitChecks());
//elevatorVoltage.setDouble(m_leftElevator.getAppliedOutput());
SmartDashboard.putNumber("Elevator Encoder", elevatorLead.getPosition().getValueAsDouble());
//SmartDashboard.putNumber("Elevator Appied Voltage", m_leftElevator.getAppliedOutput() );
}
}