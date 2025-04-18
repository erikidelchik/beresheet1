/**
 * This class represents the basic flight controller and simulation of the Bereshit spacecraft.
 * It encapsulates the simulation state and logic so that external classes (e.g., a Simulator)
 * can drive the simulation without repeating the physics code.
 *
 * Original simulation parameters and PID control logic are preserved.
 *
 * Author: ben-moshe (modified for integration)
 */
public class Beresheet_101 {
	// --- Constants (unchanged) ---
	public static final double WEIGHT_EMP = 165; // kg
	public static final double WEIGHT_FULE = 420; // kg
	public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
	// Source: https://davidson.weizmann.ac.il/online/askexpert/...
	public static final double MAIN_ENG_F = 430; // N
	public static final double SECOND_ENG_F = 25; // N
	public static final double MAIN_BURN = 0.15; // liter per sec, 12 liter per m'
	public static final double SECOND_BURN = 0.009; // liter per sec, 0.6 liter per m'
	public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;

	// --- Simulation State Variables ---
	private double vs;          // vertical speed (m/s)
	private double hs;          // horizontal speed (m/s)
	private double dist;        // remaining horizontal distance (m)
	private double ang;         // angle in degrees (0 = vertical)
	private double aboveGround; // altitude (m)
	private double time;        // simulation time (s)
	private final double dt;    // time step (s)
	private double fuel;        // fuel (liters)
	private double weight;      // current weight (kg)
	private double NN;          // throttle parameter [0,1]
	private double acc;         // current acceleration (m/s²)

	// PID controller instance (from PIDgit.java)
	private PID pid_NN;
	private PID pid_angle;


	/**
	 * Constructor: Initializes simulation state to the original values.
	 */
	public Beresheet_101() {
		// Initial conditions (as in original main method)
		vs = 24.8; //24.8
		hs = 932; //932
		dist = 181 * 1000;       // 181 km in meters
		ang = 57.8; //58.3
		aboveGround = 13748;   // altitude in meters 13748
		time = 0;
		dt = 1;                  // time step (sec)
		fuel = 121; //121
		weight = WEIGHT_EMP + fuel;
		NN = 0.7;
		acc = 0;
		pid_NN = new PID(0.04, 0.0001, 0.1, dt);
		pid_angle = new PID(0.02, 0.0001, 0.01, dt);
	}

	public static double accMax(double weight) {
		return acc(weight, true, 8);
	}

	public static double acc(double weight, boolean main, int seconds) {
		double t = 0;
		if (main) {
			t += MAIN_ENG_F;
		}
		t += seconds * SECOND_ENG_F;
		return t / weight;
	}

	public void simulateStep() {
		if ((int) time % 10 == 0 || aboveGround < 100) {
			System.out.println(
			    String.format(
			        "Time: %.2f | VS: %.2f | HS: %.2f | Dist: %.2f | Altitude: %.2f | Angle: %.2f° | Weight: %.2fkg | Acc: %.2f | Fuel: %.2fL | NN: %.2f",
			        time, vs, hs, dist, aboveGround, ang, weight, acc, fuel, NN
			    )
			);
		}

		// Check landing condition and print outcome.
		checkLanded();



		double angleDest;
		double hsKillThresholdLow = 5.0;
		double hsKillThresholdHigh = 100.0;

		// Strong angle to reduce hs quickly
		if (hs > hsKillThresholdHigh) {
			angleDest = 60.0;
		}
		else if (hs > hsKillThresholdLow) {
			// Gradually reduce angle as hs gets lower
			angleDest = 60.0 * (hs - hsKillThresholdLow) / (hsKillThresholdHigh - hsKillThresholdLow);
			angleDest = Math.max(0, Math.min(60, angleDest)); // Clamp between 0 and 60
		}
		else {
			angleDest = 0.0;
		}


		if (aboveGround < 200) {
			angleDest = 0.0;
		}


		double angleAdjustment = pid_angle.calculateAngleOutput(angleDest, ang, dt);
		ang += angleAdjustment;


		double vsDest;
		double slowdownAltitude = 1000.0; // start final slowdown below this altitude
		double finalAltitude = 5.0; //target VS until very close to ground
		double initialVSdest = 25.0; // target VS before final slowdown
		double finalVSdest = 2.0;   // target VS just before landing

		if (aboveGround > slowdownAltitude) {
			vsDest = initialVSdest;
		}
		else if (aboveGround > finalAltitude) {
			//linear func to reduce VS
			double m = (initialVSdest - finalVSdest) / (slowdownAltitude - finalAltitude);
			vsDest = finalVSdest + m * (aboveGround - finalAltitude);

			// bound values
			vsDest = Math.max(vsDest, finalVSdest);
			vsDest = Math.min(vsDest, initialVSdest);
		}
		else {
			vsDest = 1.0;
		}

		// update speed using the PID controller.
		NN -= pid_NN.calculateNNoutput(vsDest, vs, dt);

		if (NN < 0) {
			NN = 0;
		}
		if (NN > 1) {
			NN = 1;
		}

		if (aboveGround <= 3) {
			NN = 0;
		}

		// Compute acceleration components.
		double ang_rad = Math.toRadians(ang);
		double h_acc = Math.sin(ang_rad) * acc;
		double v_acc = Math.cos(ang_rad) * acc;
		double vacc = Moon.getAcc(hs);

		time += dt;
		double dw = dt * ALL_BURN * NN;
		if (fuel > 0) {
			fuel -= dw;
			weight = WEIGHT_EMP + fuel;
			acc = NN * accMax(weight);
		}
		else {
			acc = 0;
		}

		// Subtract gravity's effect from vertical acceleration.
		v_acc -= vacc;

		// Update horizontal speed.
		if (hs > 0) {
			hs -= h_acc * dt;
			if (hs < 0) {
				hs = 0;
			}
		}

		// Update state variables.
		dist -= hs * dt;
		vs -= v_acc * dt;
		aboveGround -= dt * vs;
	}

	private void checkLanded() {
		if ((vs > 2.5 || hs > 2.5) && aboveGround < 5) {
			System.out.println("crashed");
			aboveGround = 0;
		}
		else if (vs <= 2.5 && hs <= 2.5 && aboveGround <= 5) {
			System.out.println("landed");
			aboveGround = 0;
		}
	}

	// --- Getters ---
	public double getVs() { return vs; }
	public double getHs() { return hs; }
	public double getDist() { return dist; }
	public double getAng() { return ang; }
	public double getAboveGround() { return aboveGround; }
	public double getTime() { return time; }
	public double getFuel() { return fuel; }
	public  double getNN() {return  NN;}
}
