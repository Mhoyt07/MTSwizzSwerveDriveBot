// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.mod2;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  private SwerveDriveOdometry odometry;
  private Field2d field;
  private double yaw;
  private final SwerveModule[] dt;
  private final PigeonIMU gyro = new PigeonIMU(10);
  public final PIDController alignPID = new PIDController(yaw, yaw, yaw);
  
  private final Joystick driver_l;
  private final Joystick driver_r;
  
  public SwerveDrive(Joystick driver_l, Joystick driver_r) {
    //creates a "map" of the robot, recording the position of each swerve wheel

    gyro.configFactoryDefault();
    zero_gyro();

    //3, 1, 2, 0
    this.dt = new SwerveModule[] {
      new SwerveModule(3, 0,  Constants.mod3.drive_motor_id, Constants.mod3.turn_motor_id, Constants.mod3.can_coder, Constants.mod3.turning_offset),
      new SwerveModule(1, 1,  Constants.mod1.drive_motor_id, Constants.mod1.turn_motor_id, Constants.mod3.can_coder, Constants.mod1.turning_offset),
      new SwerveModule(2, 2,  Constants.mod2.drive_motor_id, Constants.mod2.turn_motor_id, Constants.mod3.can_coder, Constants.mod2.turning_offset),
      new SwerveModule(0, 3,  Constants.mod0.drive_motor_id, Constants.mod0.turn_motor_id, Constants.mod3.can_coder, Constants.mod0.turning_offset)
    };
    odometry = new SwerveDriveOdometry(Constants.swerve_map, get_yaw(), new SwerveModulePosition[] {dt[0].get_position(), dt[1].get_position(), dt[2].get_position(), dt[3].get_position()});

    this.driver_l = driver_l;
    this.driver_r = driver_r;

    Timer.delay(1);
    reset_to_absolute2();

    field = new Field2d();
  }

  public void drive(Translation2d translation, double rotation, boolean is_field_relative, boolean is_auto) {
    SmartDashboard.putNumber("Translation Angle", translation.getAngle().getDegrees());

    SwerveModuleState[] swerve_module_states = Constants.swerve_map
      .toSwerveModuleStates(is_field_relative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), translation.getY(), rotation, get_yaw())
          : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve_module_states,Constants.drive.max_speed);

    for (SwerveModule module : this.dt) {
      module.set_desired_state(swerve_module_states[module.module_pos], is_auto);
      //module_pos may need to be module_number
    }
  }

  //sets the gyro to 0 degrees
  public void zero_gyro(){
    gyro.setYaw(0);
  }

  public Rotation2d get_yaw() {
    yaw = gyro.getYaw() + Constants.drive.gyro_offset;

    while (yaw > 360) {
      yaw = yaw - 360;
    }
    return Rotation2d.fromDegrees(yaw);
  }

  public void reset_to_absolute2() {
    for (SwerveModule module : dt) {
      module.reset_to_absolute();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
