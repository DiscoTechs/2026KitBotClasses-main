

package frc.robot;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.FuelConstants.INTAKING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKING_INTAKE_VOLTAGE;
import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import limelight.Limelight;

import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotContainer {





  // ================= SUBSYSTEMS =================
  private final CANDriveSubsystem canDriveSubsystem =
      new CANDriveSubsystem();
  private final CANFuelSubsystem canFuelSubsystem = new CANFuelSubsystem();
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  // ================= CONTROLLERS =================
 //driver
  private final XboxController driverController =
      new XboxController(0);
//operator
  private final CommandXboxController operatorController = new CommandXboxController(1); //from docs




  // ================= TRIGGERS =================
  // Right trigger acts like a button when pressed > 20%
  private final Trigger rightTrigger =
      new Trigger(() -> driverController.getRightTriggerAxis() > 0.2);

  // ================= CONSTRUCTOR =================
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
  }

  // ================= DEFAULT DRIVE =================
  private void configureDefaultCommands() {
    canDriveSubsystem.setDefaultCommand(
        canDriveSubsystem.run(() ->
            canDriveSubsystem.driveArcade(
                -driverController.getLeftY(),
                driverController.getRightX()
            )
        )
    );

    //canFuelSubsystem.setDefaultCommand(new Intake(canFuelSubsystem));
  }

  // ================= BUTTON BINDINGS =================
  private void configureBindings() {

    // operatorController.rightTrigger().onTrue(Commands.runOnce(() -> shooterSubsystem.setVelocity(RPM.of(1000))));
    // operatorController.rightTrigger().onFalse(Commands.runOnce(() -> shooterSubsystem.setVelocity(RPM.of(0))));

operatorController.leftBumper().whileTrue(
    new Intake(canFuelSubsystem)
);


  
  operatorController.leftBumper().whileTrue(
    canFuelSubsystem.runEnd(
        () -> {
            canFuelSubsystem.setIntakeLauncherRoller(-INTAKING_INTAKE_VOLTAGE);
            canFuelSubsystem.setFeederRoller(-INTAKING_FEEDER_VOLTAGE);
        },
        () -> canFuelSubsystem.stop()
    )
);
  

operatorController.rightBumper().onTrue(
    new SpinUp(canFuelSubsystem)
        .withTimeout(SPIN_UP_SECONDS)
        .andThen(new Launch(canFuelSubsystem))
        .finallyDo(canFuelSubsystem::stop)
);

    // Hold RIGHT TRIGGER to use Limelight auto-centering + forward drive
   
 rightTrigger.whileTrue(
    canDriveSubsystem.run(() -> {
        if (canDriveSubsystem.shouldRunLimelightForTags()) {
            canDriveSubsystem.limelightHoldNineFeet();
        } else {
            canDriveSubsystem.driveArcade(0, 0);
        }
    })
);
}
  // ================= AUTONOMOUS =================
  public Command getAutonomousCommand() {
    return new ExampleAuto(canDriveSubsystem, canFuelSubsystem);
  }





}