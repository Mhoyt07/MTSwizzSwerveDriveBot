// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class Drive extends Command {
  private final Joystick driver_l;
  private final Joystick driver_r;

  private final SwerveDrive driver_swerve;

  private final SlewRateLimiter translation_limiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotation_limiter = new SlewRateLimiter(3);
  private final SlewRateLimiter strafe_limiter = new SlewRateLimiter(3);

  private double strafe_val;
  private double translation_val;
  private double rotation_val;
  
  private boolean is_auto;
  /** Creates a new Drive. */
  public Drive(SwerveDrive drive_swerve, Joystick driver_l, Joystick driver_r, double strafe_val, double translation_val, double rotation_val, boolean is_auto) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driver_swerve = drive_swerve;
    addRequirements(driver_swerve);

    this.driver_l = driver_l;
    this.driver_r = driver_r;

    this.strafe_val = strafe_val;
    this.translation_val = translation_val;
    this.rotation_val = rotation_val;
    
    this.is_auto = is_auto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe_value;
    double translation_value;
    double rotation_value;

    if (is_auto) {
      strafe_value = strafe_limiter.calculate(MathUtil.applyDeadband(strafe_val, 0.1));
      translation_value = translation_limiter.calculate(MathUtil.applyDeadband(translation_val, 0.1));
      rotation_value = rotation_limiter.calculate(MathUtil.applyDeadband(rotation_val, 0.1));
    } else {
      strafe_value = strafe_limiter.calculate(MathUtil.applyDeadband(this.driver_l.getRawAxis(0), 0.1));
      translation_value = translation_limiter.calculate(MathUtil.applyDeadband(this.driver_l.getRawAxis(1), 0.1));
      rotation_value = rotation_limiter.calculate(MathUtil.applyDeadband(this.driver_r.getRawAxis(0), 0.1));
    }
    driver_swerve.drive(
      new Translation2d(translation_value, strafe_value).times(Constants.drive.max_speed), 
      rotation_value * Constants.drive.max_angular_velocity, 
      true, 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
