package frc.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;



public class MotorFactory {

    private static class Config {
    }

    private static Config MASTER_CONFIG = new Config();
    private static Config SLAVE_CONFIG = new Config();

    static {
        //Any changes from talon default here
    }

    public static CANSparkMax createDefaultBrushlessNeo(int id) {
        CANSparkMax neo = new CANSparkMax(id, MotorType.kBrushless);
        neo.setSmartCurrentLimit(60);
        return neo;
    }

    public static CANSparkMax createSlavedNeo(int id, CANSparkMax master) {
        CANSparkMax neo = createDefaultBrushlessNeo(id);
        neo.follow(master);
        return neo;
    }

    public static TalonSRX createDefaultTalon(int id) {
        TalonSRX talon = new TalonSRX(id);
        talon.configFactoryDefault(Constants.Timing.CAN_TIMEOUT_MS);
        talon.configPeakCurrentLimit(60);
        talon.configContinuousCurrentLimit(60);
        return talon;
    }

    public static TalonSRX createSlavedTalon(int id, int masterId) {
        TalonSRX slave = createDefaultTalon(id);
        slave.set(ControlMode.Follower, masterId);
        return slave;
    }
    
}