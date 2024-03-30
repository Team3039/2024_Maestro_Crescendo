package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {
	public static final class Ports {
		public static final class Drive_Ports {
			public static final int FL_DRIVE_MOTOR_ID = 0;
			public static final int FL_STEER_MOTOR_ID = 1;
			public static final int FL_ENCODER_ID = 2;

			public static final int FR_DRIVE_MOTOR_ID = 3;
			public static final int FR_STEER_MOTOR_ID = 4;
			public static final int FR_ENCODER_ID = 5;

			public static final int BL_DRIVE_MOTOR_ID = 6;
			public static final int BL_STEER_MOTOR_ID = 7;
			public static final int BL_ENCODER_ID = 8;

			public static final int BR_DRIVE_MOTOR_ID = 9;
			public static final int BR_STEER_MOTOR_ID = 10;
			public static final int BR_ENCODER_ID = 11;
			public static final int PIGEON_ID = 12;
		}
		public static final int INTAKE = 13;
		public static final int INDEXER = 14;
		public static final int BEAM_BREAK = 15;
		public static final int WRIST = 16;
		public static final int SHOOTER_LEFT = 17;
		public static final int SHOOTER_RIGHT = 18;
		public static final int CLIMB = 19;
		public static final int CANdleID = 20;
		public static final int AMPER = 21;
        // public static final int CLIMB_B = 0;
	}

	public static final class Drive {
		public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
		public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed

		public static final double TRACK = Units.inchesToMeters(21.50);
		// Radians
		public static final double FL_ENCODER_OFFSET = 0.31689453125;
		public static final double FR_ENCODER_OFFSET = 0.465576171875;
		public static final double BL_ENCODER_OFFSET =  0.18603515625;
		public static final double BR_ENCODER_OFFSET = -0.414794921875;

		public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
		public static final double STEER_GEAR_RATIO = 12.8;
		public static final double WHEEL_RADIUS = 2.0; // Inches
		public static final double SPEED_AT_12_VOLTS = 5.21; // Meters Per Second
		public static final double COUPLE_RATIO = 3.5714285714285716;
		public static final double SLIP_CURRENT = 300.0; // Amps
	}

	public static final class Shooter {
		public static final double SHOOTER_KP = 0.12;
		public static final double SHOOTER_KI = 0.000;
		public static final double SHOOTER_KD = 0.000;
        public static final double SHOOTER_FF = 1.75;
        public static final double AMP_KP = 0;
        public static final double AMP_KI = 0;
        public static final double AMP_KD = 0;
		public static final float AMP_FORWARD_LIMIT = 0;
        public static final float AMP_REVERSE_LIMIT = 0;
		public static final double AMP_GEAR_RATIO = 0.1;

	}

	public static final class Vision {
		public static final double FIELD_LENGTH_METERS = 16.54175;
		public static final double FIELD_WIDTH_METERS = 8.0137;
	}

	public static final class Wrist {
		public static final double WRIST_KP = .07;
		public static final double WRIST_KI = 0;
		public static final double WRIST_KD = 0.003;
		public static final double WRIST_KS = 0.000;
		public static final double WRIST_KG = 0.000;
		public static final double WRIST_KV = 0;

		public static final double WRIST_GEAR_RATIO = 1.0 / 350.0;
		public static final float Forward_Limit = 53;
		public static final float Reverse_Limit = -27;
		public static final double WRIST_TO_AMP = 53;
        public static final double WRIST_INTAKING = 33.5;
		public static final double WRIST_TO_SOURCE = 36;
	}

	public static final class Climb{
		public static final double CLIMB_KS = 0;
        public static final double CLIMB_KP = 0;
        public static final double CLIMB_KI = 0;
        public static final double CLIMB_KD = 0;
        public static final double CLIMB_HEIGHT = 15;
	}
}
