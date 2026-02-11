package frc.robot;

import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.KrakenTurret;

import static frc.robot.Constants.FuelConstants.INTAKING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKING_INTAKE_VOLTAGE;
import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // ================= SUBSYSTEMS =================
  private final CANDriveSubsystem canDriveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem canFuelSubsystem = new CANFuelSubsystem();
  private final KrakenTurret krakenTurret = new KrakenTurret(3);

  // ================= CONTROLLERS =================
  private final XboxController driverController = new XboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // ================= TRIGGERS =================
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
  }

  // ================= BUTTON BINDINGS =================
  private void configureBindings() {

    // Intake (operator LB)
    operatorController.leftBumper().whileTrue(new Intake(canFuelSubsystem));

    operatorController.leftBumper().whileTrue(
        canFuelSubsystem.runEnd(
            () -> {
              canFuelSubsystem.setIntakeLauncherRoller(-INTAKING_INTAKE_VOLTAGE);
              canFuelSubsystem.setFeederRoller(-INTAKING_FEEDER_VOLTAGE);
            },
            canFuelSubsystem::stop
        )
    );

    // Shoot sequence (operator RB)
    operatorController.rightBumper().onTrue(
        new SpinUp(canFuelSubsystem)
            .withTimeout(SPIN_UP_SECONDS)
            .andThen(new Launch(canFuelSubsystem))
            .finallyDo(canFuelSubsystem::stop)
    );

    // Kraken turret controls
    configureKrakenTurretBindings();

    // Limelight auto drive
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

  // ================= KRAKEN TURRET BINDINGS =================
  private void configureKrakenTurretBindings() {

    // RB → turret right
    operatorController.rightTrigger()
        .whileTrue(Commands.run(() -> krakenTurret.run(0.4), krakenTurret))
        .onFalse(Commands.runOnce(krakenTurret::stop, krakenTurret));

    // LB → turret left
    operatorController.leftTrigger()
        .whileTrue(Commands.run(() -> krakenTurret.run(-0.4), krakenTurret))
        .onFalse(Commands.runOnce(krakenTurret::stop, krakenTurret));
  }

  // ================= AUTONOMOUS =================
  public Command getAutonomousCommand() {
    return new ExampleAuto(canDriveSubsystem, canFuelSubsystem);
  }
}