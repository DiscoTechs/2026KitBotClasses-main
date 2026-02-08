// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Pounds;
// import static edu.wpi.first.units.Units.RPM;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import yams.gearing.GearBox;
// import yams.gearing.MechanismGearing;
// import yams.mechanisms.SmartMechanism;
// import yams.mechanisms.config.FlyWheelConfig;
// import yams.mechanisms.velocity.FlyWheel;
// import yams.motorcontrollers.SmartMotorController;
// import yams.motorcontrollers.SmartMotorControllerConfig;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;
// import yams.motorcontrollers.remote.TalonFXWrapper;

// public class ShooterSubsystem extends SubsystemBase {
//   private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   .withClosedLoopController(1, 0, 0)
//   .withSimClosedLoopController(1, 0, 0)
//   .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
//   .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
//   .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.COAST)
//   .withStatorCurrentLimit(Amps.of(40));

//   private TalonFX talon = new TalonFX(2);
//   private SmartMotorController sparkSmartMotorController = new TalonFXWrapper(talon, DCMotor.getNEO(1), smcConfig);

//  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
//   .withDiameter(Inches.of(4))
//   .withMass(Pounds.of(1))
//   .withUpperSoftLimit(RPM.of(1000))
//   .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

//   private FlyWheel shooter = new FlyWheel(shooterConfig);

//   /**
//    * Gets the current velocity of the shooter.
//    *
//    * @return Shooter velocity.
//    */
//   public AngularVelocity getVelocity() {return shooter.getSpeed();}

//   /**
//    * Set the shooter velocity.
//    *
//    * @param speed Speed to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}
  
//   /**
//    * Set the shooter velocity setpoint.
//    *
//    * @param speed Speed to set
//    */
//   public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}

//   /**
//    * Set the dutycycle of the shooter.
//    *
//    * @param dutyCycle DutyCycle to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

//   public ShooterSubsystem() {}

//   @Override
//   public void periodic() { shooter.updateTelemetry(); }

//   @Override
//   public void simulationPeriodic() { shooter.simIterate(); }
// }
