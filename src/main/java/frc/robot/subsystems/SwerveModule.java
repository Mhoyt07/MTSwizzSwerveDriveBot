// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.CANSparkMaxUtil.Usage;
import frc.robot.CANSparkMaxUtil;

/** Add your docs here. */
public class SwerveModule {
    public final int module_number;
    
    public final Rotation2d turning_offset;
    public final int module_pos;

    //creating motors and encoders using canspark
    private final CANSparkMax drive_motor;
    private final CANSparkMax turn_motor;

    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;

    private final CANCoder turning_can_coder;

    private final PIDController turnPID;
    private final PIDController drivePID;

    private final LinearFilter drive_vel_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter desired_drive_filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private double drive_velocity;
    private double desired_drive_vel;

    //skeleton of each swerve module
    public SwerveModule(int module_number, int module_pos, int drive_motor_id, int turn_motor_id, int can_coder_id, Rotation2d turning_offset) {
        this.module_number = module_number;
        this.turning_offset = turning_offset;
        this.module_pos = module_pos;
        
        this.turning_can_coder = new CANCoder(can_coder_id);

        this.turn_motor = new CANSparkMax(turn_motor_id, MotorType.kBrushless);
        this.turn_motor.restoreFactoryDefaults(); //restores factory defaults
        this.turn_motor.setIdleMode(IdleMode.kBrake);
        this.turn_motor.setInverted(true); //inverts motor direction
        this.turn_motor.setSmartCurrentLimit(20);
        this.turn_motor.enableVoltageCompensation(12);
        this.turn_motor.burnFlash();

        //sets up encoder
        this.turn_encoder = this.turn_motor.getEncoder();
        this.turn_encoder.setPositionConversionFactor(Constants.drive.turn_motor_pos_factor);

        this.turnPID = new PIDController(Constants.drive.turnkP, Constants.drive.turnkI, Constants.drive.turnkD);

        CANSparkMaxUtil.setCANSparkMaxBusUsage(this.turn_motor, Usage.kPositionOnly);



        //drive motor definition
        this.drive_motor = new CANSparkMax(drive_motor_id, MotorType.kBrushless);
        this.drive_motor.restoreFactoryDefaults();
        this.drive_motor.setSmartCurrentLimit(50);
        this.drive_motor.enableVoltageCompensation(12);
        this.drive_motor.setIdleMode(IdleMode.kBrake);
        this.drive_motor.setInverted(false);
        this.drive_motor.burnFlash();

        this.drive_encoder = this.drive_motor.getEncoder();
        this.drive_encoder.setPositionConversionFactor(Constants.drive.drive_motor_pos_factor);
        this.drive_encoder.setVelocityConversionFactor(Constants.drive.drive_motor_vel_factor);
        this.drive_encoder.setPosition(0);

        this.drivePID = new PIDController(Constants.drive.drivekP, Constants.drive.drivekI, Constants.drive.drivekD);
    }

    public SwerveModuleState get_state() {
        return new SwerveModuleState(this.drive_encoder.getVelocity(), this.get_cancoder());
    }

    //retursn position
    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(this.drive_encoder.getPosition(), new Rotation2d(turn_encoder.getPosition()));
    }

    public Rotation2d get_cancoder() {
        return Rotation2d.fromDegrees(this.turning_can_coder.getAbsolutePosition());
    }

    public void reset_to_absolute() {
        System.out.print(get_cancoder().getDegrees());

        this.turn_encoder.setPosition(turning_offset.getDegrees());
    }

    public void set_desired_state(SwerveModuleState desired_state, boolean is_auto) {
        boolean invert_drive_motor = set_angle(desired_state);
        if(is_auto) {
            new WaitCommand(2);
        }

        set_speed(desired_state, is_auto, invert_drive_motor);
    }

    public void set_speed(SwerveModuleState desired_state, boolean is_auto, boolean invert_drive_motor) {
        if (is_auto == false) {
            double drive_motor_output = desired_state.speedMetersPerSecond / Constants.drive.max_speed;
            drive_motor.set(invert_drive_motor ? drive_motor_output * -1 : drive_motor_output);
        }
        else {
            drive_velocity = drive_vel_filter.calculate(drive_encoder.getVelocity());
            desired_drive_vel = desired_drive_filter.calculate(desired_state.speedMetersPerSecond);

            double drive_auto_motor_voltage = drivePID.calculate(drive_velocity, desired_drive_vel);

            drive_motor.setVoltage(invert_drive_motor ? drive_auto_motor_voltage * -1 : drive_auto_motor_voltage);
        }
    }

    private boolean set_angle(SwerveModuleState desired_state) {
        boolean invert_drive_motor = false;

        SwerveModuleState current_state = this.get_state();

        double current_degrees = (current_state.angle.getDegrees() - this.turning_offset.getDegrees());

        current_degrees = current_degrees < -180 ? current_degrees + 360 : current_degrees;
        current_degrees = current_degrees > 180 ? current_degrees - 360 : current_degrees;

        double desired_degrees = desired_state.angle.getDegrees() % 360;

        double diff = (current_degrees - desired_degrees + 180) % 360 - 180;
        diff = diff < -180 ? diff + 360 : diff;
        diff = diff > 180 ? diff - 360 : diff;


        if (Math.abs(diff) > 90) {
            invert_drive_motor = true;
            if (diff < 0) {
                diff += 180;
            } else {
                diff -= 180;
            }
        }


        double turn_motor_value = Math.abs(diff) < 1 ? 0 : turnPID.calculate(diff, 0);

        turn_motor_value = turn_motor_value > 1 ? 1 : turn_motor_value;
        turn_motor_value = turn_motor_value < -1 ? -1 : turn_motor_value;

        turn_motor.set(turn_motor_value);

        return invert_drive_motor;
    }
}
