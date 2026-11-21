// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class IntakeConstants{

    public static final double ROLLER_VOLTS = 3.0; // tune this later
    

     //pivot degs
    public static final double PIVOT_GEAR_RATIO = (38.0 / 12.0) * (50.0 / 16.0) * (44.0 / 12.0); 
    public static final double STOWED_DEGS = 0.0;
    public static final double DEPLOYED_DEGS = -127.0;
    public static final double DEGS_TO_MOTOR_ROTS = PIVOT_GEAR_RATIO/ 360.0;
 
  //canids
    public static final int PIVOT_LEFT_ID = 31;

    public static final int ROLLER_LEFT_ID = 33;
    public static final int ROLLER_RIGHT_ID = 34;

 
  }

 

  public static final class DriveConstants{
    public static final double DRIVE_GEARING = 6.75;
    public static final double STEER_GEARING = 150.0 / 7.0;
    public static final double WHEEL_DISTANCE_PER_TURN = Math.PI * Units.inchesToMeters(4.0); 

    public static final double MAX_SPEED = 4.5;                  // meters per second. TODO
    public static final double MAX_TURN_SPEED = 4.7;             // radians per second. TODO
    public static final double ROBOT_WIDTH = Units.inchesToMeters(22.5);  // wheel to wheel, left to right. TODO
    public static final double ROBOT_LENGTH = Units.inchesToMeters(22.5); // wheel to wheel, front to back. TODO
    public static final int GYRO_ID = 10;   

    
  }
}
 
