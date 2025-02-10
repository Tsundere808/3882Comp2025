// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class CoralIntakeConstants{
    public static final int intakeID = 31;
    public static final int intakeSpeed = -20;
    public static final int outtakeSpeed = 20;

  }
  public static final class ShooterConstants {
    public static final int topShooter = 34;
    public static final int botShooter = 33;
    public static final int shootSpeed = 10;
}
public static final class FeederConstants {
  public static final int feederID = 32;
  public static final int feederEncoderA = 5;
  public static final int feederEncoderB = 6;
  public static final double IntakeSPEED = .2;
  public static final double OutakeSPEED = -.2;
  public static final double ampOutakeSPEED = 60;
}
public static final class PivotConstants {
        
  public static final int pivot = 40;
  public static final int pivotspeed = 20;
  public static final double homePosition = 0.1;
  public static final int intakePosition = -10;
  public static final int subwooferShotPosition = -19;
  public static final int farShot = -29;
  public static final int extremeFarShot = -10;
  public static final int elevatorSubwooferShotPosition = -10;
  public static final int elevatorFarShot = -10;
  public static final double PIVOTMAX = -18.5;

}
public static final class ElevatorConstants {
        
  public static final int leftElevator = 14;
  public static final double eHomePos = 0;
  public static final double eAmp = -50;
  public static final double efarshot = -50;
  public static final double eextremefarshot = -50;
  public static final double eClimbPos = -25;
  public static final double ELEVATORMAX = -50.21;

}
public static final class ClimberConstants
{
  public static final int climber = 41;
  public static final int cHomePos = 0;
  public static final int cUpPose = 61;
  public static final int cClimbPos = 100; 
  
}
}
