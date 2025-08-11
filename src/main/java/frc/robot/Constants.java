// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // TODO: Insert DriveConstants here...
  public static class DriveConstants {
    public static final int kLeftLeaderCANId = 0;
    public static final int kLeftFollowerCANId = 1;
    public static final int kRightLeaderCANId = 2;
    public static final int kRightFollowerCANId = 3;
  }
  // Arm Constants
  public static class ArmConstants {
    public final static int canIDCanCoder = 11;
    public final static int krakenCanID = 12;
    public final static double gearRatio = 25; //25:1 gear ratio
    public final static double kP = 100;
    public final static double kI = 2;
    public final static double kD = 1;
    public final static double maxVelocity = 1; // rad/s
    public final static double maxAcceleration = 1; // rad/s²
    public final static boolean brakeMode = true;
    public final static boolean enableStatorLimit = true;
    public final static double statorCurrentLimit = 40;
    public final static boolean enableSupplyLimit = false;
    public final static double supplyCurrentLimit = 40;
    public final static double armLength = 0.5; // meters
    public final static double minAngleDeg = 0;
    public final static double maxAngleDeg = 90;
    public final static double kS = 0;
    public final static double kG = 0;
    public final static double kA = 0;
    public final static double kV = 0;
  } 
  // Elevator constants for elevator subsystem
  public static final class ElevatorConstants {
    // CAN IDs
    public static final int kElevatorLeaderCAN = 54;
    public static final int kElevatorFollowerCAN = 55;

    // Physical Constants
    public static final double kElevatorGearing = 7.5;
    public static final double kElevatorCarriageMass = Units.lbsToKilograms(20);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.644 / 2);
    public static final double kElevatorMetersPerMotorRotation = (kElevatorDrumRadius * 2 * Math.PI)
            / kElevatorGearing;

    // Elevator Dimensions
    public static final double kElevatorHeightMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.90;

    // Motion Constraints
    public static final double kElevatorMaxVelocity = 1.5 / ElevatorConstants.kElevatorMetersPerMotorRotation;
    public static final double kElevatorMaxAcceleration = 160.0;

    // PID Constants
    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.01;

    // Elevator Gains
    // set all to 0 during testing
    public static final double kElevatorS = 0.0; // negligible
    public static final double kElevatorG = 0.2;
    public static final double kElevatorV = 6.85 * kElevatorMetersPerMotorRotation;
    public static final double kElevatorA = 0.04 * kElevatorMetersPerMotorRotation;

    // ===== Elevator Setpoints =====
    public static final double kElevatorCoralStationAndProcessorHeight = 0.0;

    public static final double kElevatorCoralLevel1StartHeight = 0.025;
    public static final double kElevatorCoralLevel1EndHeight = 0.225;
    public static final double kElevatorCoralLevel2Height = 0.188;
    public static final double kElevatorCoralLevel3Height = 0.548;

    public static final double kElevatorAlgaeLowHeight = 0.604;
    public static final double kElevatorAlgaeHighHeight = 0.90;
    // ==============================

    // ===== Control Parameters =====
    public static final double kElevatorTargetError = 0.005;
    public static final double kElevatorMotorResistance = 0.002; // Assume 2mOhm resistance for
                                                                 // voltage drop calculation
  }
}
