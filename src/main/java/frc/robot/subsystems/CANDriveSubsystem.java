
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTableInstance;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
 
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);

  }

  public void limelightDrive() {
    double tx = LimelightHelpers.getTX("limelight");
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (!hasTarget) {
        drive.arcadeDrive(0.0, 0.0);
        return;

    }
    // ---- Tune these ----
    double kTurn = 0.03;   //started at 0.03
    double forwardSpeed = 0.4; // started at 0.4 

    double turn = tx * kTurn;

    drive.arcadeDrive(forwardSpeed, turn);

  }



public void limelightDriveReverse() {
    double tx = NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("tx")
        .getDouble(0);

    double kP = 0.03;   // tune later
    double turn = -tx * kP;

    driveArcade(-0.4, turn);
}



public void limelightHoldNineFeet() {
    var table = NetworkTableInstance.getDefault().getTable("limelight");

    // Check target valid
    double tv = table.getEntry("tv").getDouble(0);
    if (tv != 1) {
        driveArcade(0, 0);
        return;
    }

    // Distance to tag (meters)
    double[] botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    double distanceMeters = botPose[2];

    // Constants
    double targetMeters = 2.7432; // 9 feet
    double tolerance = 0.05;      // ~2 inches
    double kP = 0.8;              // distance gain
    double maxSpeed = 0.4;

    // Distance control
    double error = targetMeters - distanceMeters;
    double forward = error * kP;

    // Deadband (stop exactly at 9 ft)
    if (Math.abs(error) < tolerance) {
        forward = 0;
    }

    // Clamp speed
    forward = Math.max(-maxSpeed, Math.min(maxSpeed, forward));

    // Steering (tx)
    double tx = table.getEntry("tx").getDouble(0);
    double turn = -tx * 0.03;

    driveArcade(forward, turn);
}





public boolean shouldRunLimelightForTags() {
    double tv = NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("tv")
        .getDouble(0);

    if (tv != 1) return false;

    int tagId = (int) NetworkTableInstance.getDefault()
        .getTable("limelight")
        .getEntry("tid")
        .getDouble(-1);

    return tagId == 10 || tagId == 25;
}

}