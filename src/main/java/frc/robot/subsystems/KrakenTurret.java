package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenTurret extends SubsystemBase {

    private final TalonFX kraken;
    private final DutyCycleOut output = new DutyCycleOut(0);

    public KrakenTurret(int canID) {
        kraken = new TalonFX(canID);

        // 🔒 Brake mode (IMPORTANT)
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        kraken.getConfigurator().apply(motorConfigs);
    }

    /** Rotate turret (open-loop) */
    public void run(double speed) {
        output.Output = speed;
        kraken.setControl(output);
    }

    /** Stop turret (brake mode will hold it) */
    public void stop() {
        run(0);
    }
}