// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

  //swerve druve constants
  //module 0 constants (front right)
  public static class mod0 {
    public static final int drive_motor_id = 54;
    public static final int turn_motor_id = 58;
    public static final int can_coder = 5;
    public static final Rotation2d turning_offset = Rotation2d.fromDegrees(67.3);
  }
  //module 1 constants (back right)
  public static class mod1 {
    public static final int drive_motor_id = 36;
    public static final int turn_motor_id = 52;
    public static final int can_coder = 6;
    public static final Rotation2d turning_offset = Rotation2d.fromDegrees(271.2);
  }
  //module 2 constants (back left)
  public static class mod2 {
    public static final int drive_motor_id = 53;
    public static final int turn_motor_id = 61;
    public static final int can_coder = 4;
    public static final Rotation2d turning_offset = Rotation2d.fromDegrees(103.8);
  }
  //module 3 constants (front left)
  public static class mod3 {
    public static final int drive_motor_id = 59;
    public static final int turn_motor_id = 55;
    public static final int can_coder = 7;
    public static final Rotation2d turning_offset = Rotation2d.fromDegrees(226.0);
  }


  //drivetrain general constants
  public static class drive {
    //gyro offset in degrees
    public static final double gyro_offset = 90;

    //gear ratios (for the convefsion factors)
    public static final double drive_motor_ratio = 6.12;
    public static final double turn_motor_ratio = 150/7;

    //conversion factors
    public static final double drive_motor_pos_factor = Math.PI * drive_motor_ratio * 9.5;
    public static final double drive_motor_vel_factor = drive_motor_pos_factor / 60;
    public static final double turn_motor_pos_factor = 360 / turn_motor_ratio;

    //max drive of the robot ( in meters per second or radians per second)
    public static final double max_speed = 5;
    public static final double max_angular_velocity = 7.0;

    //length of drivetrain, as well as the map of where the wheels are on the robot (like coordinates)
    public static final double robot_length = Units.inchesToMeters(0);
    public static final double robot_width = Units.inchesToMeters(0);

    //module pid constants
    public static final double drivekP = 0.02;
    public static final double drivekI = 0;
    public static final double drivekD = 0;

    public static final double turnkP = 0.00759; 
    public static final double turnkI = 0.00069;
    public static final double turnkD = 0.0001; 

  }

  //swerve drive kinematics
  public static final SwerveDriveKinematics swerve_map = new SwerveDriveKinematics(
    new Translation2d(drive.robot_length / 2, drive.robot_width / 2), //++
    new Translation2d(drive.robot_length / 2, -drive.robot_width / 2), //+-
    new Translation2d(-drive.robot_length / 2, drive.robot_width / 2), //-+
    new Translation2d(-drive.robot_length / 2, -drive.robot_width / 2) //--
  );
}
