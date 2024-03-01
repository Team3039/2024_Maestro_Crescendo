package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final class Drive{
	}

	public static final class Elevator {
		// Elevator PID //
		public static final double ELEVATOR_KP = 0.05;
		public static final double ELEVATOR_KI = 0;
		public static final double ELEVATOR_KD = 0;

		// Elevator FF //
		public static final double ELEVATOR_KS = 0.032;
		public static final double ELEVATOR_KG = 0;
		public static final double ELEVATOR_KV = 0;
		public static final double ELEVATOR_MAX_VEL = 0;
		public static final double ELEVATOR_MAX_ACCEL = 0;
	}

	public static final class Shooter{
		public static int ShooterLeft = 0;
		public static int ShooterRight = 1;
	}

	public static final int CANdleID = 0;

	public static final class Vision {
		public static final Transform3d shootCameraToRobot = new Transform3d(
				new Translation3d(Units.inchesToMeters(0), 0, 0),
				new Rotation3d(0, Units.degreesToRadians(0), 0));
		public static final Transform3d intakeCameraToRobot = new Transform3d(
				new Translation3d(Units.inchesToMeters(0), 0, 0),
				new Rotation3d(0, Units.degreesToRadians(0), 0));

		public static final double X_P = 2.7;
		public static final double X_I = 0.0;
		public static final double X_D = 0;

		public static final double Y_P = 2.7;
		public static final double Y_I = 0.0;
		public static final double Y_D = 0;

		public static final double THETA_P = 2;
		public static final double THETA_I = 0.0;
		public static final double THETA_D = 0;

		public static final double X_TOLLERENCE = 0.01;
		public static final double Y_TOLLERENCE = 0.02;
		public static final double THETA_TOLLERENCE = 0.02;

		public static final double FIELD_LENGTH_METERS = 16.54175;
		public static final double FIELD_WIDTH_METERS = 8.0137;

	}
	public static final class Intake{

	}
	public static final class Indexer{
		public static int BeamBreak1 = 0;
		public static int Indexer = 0;

	}
	public static final class Wrist {
		public static final double WRIST_KP = 0.000;
		public static final double WRIST_KI = 0;
		public static final double WRIST_KD = 0;
		public static final double WRIST_KS = 0.000;
		public static final double WRIST_KG = 0.000;
		public static final double WRIST_KV = 0;

		public static final double WRIST_GEAR_RATIO = 0;
		public static final float Forward_Limit = 30;
        public static final float Reverse_Limit = 0;
		public static final double K_P = 0;
        public static final double K_I = 0;
        public static final double K_D = 0;
	}
}
