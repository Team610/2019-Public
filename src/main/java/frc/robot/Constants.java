package frc.robot;

import java.util.HashMap;

import frc.robot.subsystems.ArmAndManipulatorState;
import frc.robot.subsystems.Manipulator.ManipState;
import frc.util.LinearRegression;
import frc.util.Point;

/**
 * Presets
 * Vision
 * Auto
 */


/**
 * All units are in SI (meters, kilograms, seconds) unless otherwise specified
 */
public class Constants {

    public class Timing {
        public static final double LOOP_PERIOD = 0.010;
        public static final int CAN_TIMEOUT_MS = 10;
    }

    public class Vision {
        public static final double X2_DIST = 3.3102;
        public static final double X_DIST = -32.857;
        public static final double INT_DIST = 80.65;
    }

    public class Logger {
        public static final String FILE_PATH = "/home/lvuser/crash_tracking.txt";
        public static final String NAMED_FILE_PATH = "/home/lvuser/";
    }

    public class DriveTrain {
        //Ports
        public static final int LEFT_A = 3;
        public static final int LEFT_MASTER = 1;
        public static final int RIGHT_A = 4;
        public static final int RIGHT_MASTER = 2;
        // public static final int LIGHT = 7;

        //Constants
        public static final double STRAIGHT_KP = -3.4;
        public static final double STRAIGHT_KI = 0;
        public static final double STRAIGHT_KD = 0;
        public static final double TURN_KP = 0.006;
        public static final double TURN_KI = 0;
        public static final double TURN_KD = 0;

        //Dimensions
        public static final double LENGTH = 33.25 * Units.METERS_PER_INCH;
        public static final double WIDTH = 26.75 * Units.METERS_PER_INCH;
        public static final double WHEEL_DIAMETER_METERS = 6 * Units.METERS_PER_INCH;
        public static final double MOTOR_TO_WHEEL_RATIO = 13;
    }

    public static class Arm {
        //Ports
        public static final int ARM_MASTER = 12;
        public static final int ARM_SLAVE = 11;
        public static final int EXTENSION = 6;
        public static final int POTENTIOMETER = 0;
        public static final int BIKE_BRAKE_FORWARD = 0;
        public static final int BIKE_BRAKE_REVERSE = 1;

        //Constants
        public static final double ROTATION_KP = -1.4;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;
        public static final double EXTENSION_KP = 15;
        public static final double EXTENSION_KI = 0;
        public static final double EXTENSION_KD = 0;
        public static final double PARALLEL_KP = -0.15;
        public static final double PARALLEL_KI = 0.0;
        public static final double PARALLEL_KD = 0.0;

        private static final LinearRegression angleLinearRegression = new LinearRegression(new double[]{-246, -493, -748}, new double[]{0, Math.PI/2, Math.PI});

        public static final double EXTENSION_TICK_TO_METERS_SLOPE = 6.23184E-06;
        public static final double EXTENSION_TICK_TO_METERS_INTERCEPT = 0.551411199;
        public static final double ANGLE_TICK_TO_RAD_SLOPE = angleLinearRegression.slope();
        public static final double ANGLE_TICK_TO_RAD_INTERCEPT = angleLinearRegression.intercept();

        //Dimensions
        public static final int FORWARD_SOFT_LIMIT = 161572 + 8200 + 4000 + 4100; //TODO: What does this number mean
        public static final double DIST_TO_FRONT = DriveTrain.LENGTH / 2;
        public static final double SHORTEST_LENGTH = 0.551411199;
        public static final double MAXIMUM_EXTENDED_DIST = 22 * Units.METERS_PER_INCH;
        public static final double ANGLE_TOLERANCE = 4 * Units.RADIANS_PER_DEGREE;
        /** The +- angle where the bike brake is triggered to hold position */
        public static final double ANGLE_BRAKE_TOLERANCE = 2 * Units.RADIANS_PER_DEGREE;
        public static final double MAX_FORWARD_SAFE_ANGLE = -183 * ANGLE_TICK_TO_RAD_SLOPE + ANGLE_TICK_TO_RAD_INTERCEPT;
        public static final double MAX_REVERSE_SAFE_ANGLE = 3.382;
        public static final double POTENTIOMETER_FORWARD = 1579;
        public static final double POTENTIOMETER_BACKWARDS = 3499;
    }

    public static class Manipulator {
        //Ports
        public static final int WRIST = 9;
        public static final int ROLLER_A = 10;
        public static final int ROLLER_B = 8;
        public static final int HATCH_FORWARD = 4;
        public static final int HATCH_REVERSE = 5;
        public static final int LEFT_EYE = 1;
        public static final int RIGHT_EYE = 0;

        //Constants
        public static final double BALL_STALL_SPEED = 0.18;
        public static final double KP = -1.3;
        public static final double KI = 0;
        public static final double KD = 0;
        private static final LinearRegression angleTickToRadRegression = new LinearRegression(new double[]{737, 495, 229}, new double[]{Math.PI/2, 0, -Math.PI/2});
        public static final double ANGLE_TICK_TO_RAD_SLOPE = angleTickToRadRegression.slope();
        public static final double ANGLE_TICK_TO_RAD_INTERCEPT = angleTickToRadRegression.intercept();
 
        //Dimensions
        public static final double MAX_FORWARD_VALUE = 235; //~-90deg
        public static final double MAX_REVERSE_VALUE = 740; //~90deg
		public static final double LENGTH = 0.28616;
    }

    public class Stilts {
        //Ports
        public static final int MOTOR = 7;
        public static final int QUAD_FORWARD = 2;
        public static final int QUAD_REVERSE = 3;
        public static final int MAX_SAFE_EXTENSION = 28000;
    }

    public class RGB {
        public static final int CANIFIER = 15;
    }

    public class Units {
        public static final double METERS_PER_INCH = 0.0254;
        public static final double RADIANS_PER_DEGREE = Math.PI/180; 
        public static final double EPSILON = 1e-2; 
    }
    
    public class Controls {
        public static final int DRIVE_EXPONENTIAL = 2;
        public static final double JOYSTICK_THRESHOLD = 0.02;
    }

    public static class Presets {
        private static final HashMap<String, ArmAndManipulatorState> hashPresets = new HashMap<>();
        static {
            /**FLAGS
             * X and Y are offsets from the POTENTIOMETER on the ARM
             * The angle for the WRIST is the same angle geometry as the ARM
             * AUTO_GUESS - calculates the angle for the manupulator to be perpendicular to the floor at a given x,y coordinate
             * AUTO - sets the wanted angle to be perpendicular to the floor as the ARM is rotating
             * ANGLE_LOCK - the angle of the manipulator given in radians relative to the arm
            //1. "ROC" or "CRS" or "INT" or "CEN" or "STC"
            //2. "FOR" or "REV"
            //3. "CAR" or "HAT"
            //4. "TOP" or "MID" or "LOW"
            */
            hashPresets.put("INT_FOR_BAL", new ArmAndManipulatorState(new Point(Arm.SHORTEST_LENGTH + 4 * Units.METERS_PER_INCH, 0.02), ManipState.ANGLE_LOCK, 350 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));
            hashPresets.put("INT_REV_BAL", new ArmAndManipulatorState(new Point(-(Arm.SHORTEST_LENGTH + 4 * Units.METERS_PER_INCH), 0.02), ManipState.ANGLE_LOCK, 665 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));
            hashPresets.put("INT_FOR_HAT", new ArmAndManipulatorState(new Point(Arm.SHORTEST_LENGTH, 6 * Units.METERS_PER_INCH - 0.24), ManipState.ANGLE_LOCK, 480 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));
            hashPresets.put("INT_REV_HAT", new ArmAndManipulatorState(new Point(-(Arm.SHORTEST_LENGTH), 0.31), ManipState.AUTO_GUESS, 607 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));

            hashPresets.put("ROC_FOR_BAL_TOP", new ArmAndManipulatorState(new Point((DriveTrain.LENGTH/2 + 0.04 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.68 + 0.73), ManipState.ANGLE_LOCK, 320 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));
            hashPresets.put("ROC_REV_BAL_TOP", new ArmAndManipulatorState(new Point(-(DriveTrain.LENGTH/2 + 0.04 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.68 + 0.73), ManipState.ANGLE_LOCK, -(320 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT)));
            hashPresets.put("ROC_FOR_BAL_MID", new ArmAndManipulatorState(new Point((DriveTrain.LENGTH/2 + 0.09 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.45), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_REV_BAL_MID", new ArmAndManipulatorState(new Point(-(DriveTrain.LENGTH/2 + 0.09 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.45), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_FOR_BAL_LOW", new ArmAndManipulatorState(new Point((Arm.SHORTEST_LENGTH), 0.3), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_REV_BAL_LOW", new ArmAndManipulatorState(new Point(-(Arm.SHORTEST_LENGTH), 0.3), ManipState.AUTO_GUESS, 0));

            hashPresets.put("ROC_FOR_HAT_TOP", new ArmAndManipulatorState(new Point(DriveTrain.LENGTH/2 + 0.04 - Manipulator.LENGTH, 0.88 - 9 * Units.METERS_PER_INCH + 0.68), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_REV_HAT_TOP", new ArmAndManipulatorState(new Point(-(DriveTrain.LENGTH/2 + 0.04 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.68 + 0.7), ManipState.ANGLE_LOCK, Math.PI/2 - 30 * Units.RADIANS_PER_DEGREE));
            hashPresets.put("ROC_FOR_HAT_MID", new ArmAndManipulatorState(new Point(DriveTrain.LENGTH/2 + 0.07 - Manipulator.LENGTH, 0.88 - 9 * Units.METERS_PER_INCH), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_REV_HAT_MID", new ArmAndManipulatorState(new Point(-(DriveTrain.LENGTH/2 + 0.09 - Manipulator.LENGTH), 0.88 - 9 * Units.METERS_PER_INCH + 0.45), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_FOR_HAT_LOW", new ArmAndManipulatorState(new Point(Arm.SHORTEST_LENGTH, 6 * Units.METERS_PER_INCH - 0.25), ManipState.AUTO_GUESS, 0));
            hashPresets.put("ROC_REV_HAT_LOW", new ArmAndManipulatorState(new Point(-(Arm.SHORTEST_LENGTH), 0.3), ManipState.AUTO_GUESS, 0));

            hashPresets.put("CRS_FOR_HAT", new ArmAndManipulatorState(new Point(Arm.SHORTEST_LENGTH, 6 * Units.METERS_PER_INCH - 0.25), ManipState.AUTO_GUESS, 0));
            hashPresets.put("CRS_REV_HAT", new ArmAndManipulatorState(new Point(-(Arm.SHORTEST_LENGTH), 0.3), ManipState.AUTO_GUESS, 0));
            hashPresets.put("CRS_FOR_BAL", new ArmAndManipulatorState(new Point((0.4), Arm.SHORTEST_LENGTH + 0.3 - 4 * Units.METERS_PER_INCH), ManipState.AUTO_GUESS, 0));
            hashPresets.put("CRS_REV_BAL", new ArmAndManipulatorState(new Point(-(0.4), Arm.SHORTEST_LENGTH + 0.3 - 4 * Units.METERS_PER_INCH), ManipState.AUTO_GUESS, 0));
            hashPresets.put("CEN", new ArmAndManipulatorState(new Point(0, Arm.SHORTEST_LENGTH), ManipState.ANGLE_LOCK, 0));//370 * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT));
            hashPresets.put("STC", new ArmAndManipulatorState(new Point(0, Arm.SHORTEST_LENGTH), ManipState.ANGLE_LOCK, 0)); //TODO: Find
        }

        public static ArmAndManipulatorState getState(String flag) {
            if(flag.substring(0, 3).equals("CEN")) {
                return hashPresets.get("CEN");
            }
            if(flag.substring(0, 3).equals("STC")) {
                return hashPresets.get("STC");
            }
            if(flag.substring(0, 3).equals("ROC")) {
                return hashPresets.get(flag);
            }
            if(flag.substring(0, 3).equals("INT") || flag.substring(0, 3).equals("CRS")) {
                return hashPresets.get(flag.substring(0, 11));
            }
            return hashPresets.get(flag);
        }

    }

    public class XBox360 {
        public static final int BTN_X = 3;
        public static final int BTN_A = 1;
        public static final int BTN_B = 2;
        public static final int BTN_Y = 4;
        public static final int BTN_R1 = 6;
        public static final int AXIS_R2 = 3;
        public static final int BTN_L1 = 5;
        public static final int AXIS_L2 = 2;
        public static final int BTN_BACK = 7;
        public static final int BTN_START = 8;
        
        public static final int AXIS_LEFT_X = 0;
        public static final int AXIS_LEFT_Y = 1;
        public static final int AXIS_RIGHT_X = 4;
        public static final int AXIS_RIGHT_Y = 5;
        
        public static final int BTN_LEFT_JOYSTICK = 9;
        public static final int BTN_RIGHT_JOYSTICK = 10;
    }  

}