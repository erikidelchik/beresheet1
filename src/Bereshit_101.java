/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 *
 */
public class Bereshit_101 {
	public static final double WEIGHT_EMP = 165; // kg
	public static final double WEIGHT_FULE = 420; // kg
	public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
// https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
	public static final double MAIN_ENG_F = 430; // N
	public static final double SECOND_ENG_F = 25; // N
	public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
	public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
	public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;

	public static double accMax(double weight) {
		return acc(weight, true,8);
	}

	//main - main engine
	//seconds - secondary engines
	public static double acc(double weight, boolean main, int seconds) {
		double t = 0;
		if(main) {t += MAIN_ENG_F;}
		t += seconds*SECOND_ENG_F;
		double ans = t/weight;
		return ans;
	}
	// 14095, 955.5, 24.8, 2.0
	public static void main(String[] args) {
		System.out.println("Simulating Bereshit's Landing:");
		// starting point:
		double vs = 54.8;  //vertical speed
		double hs = 932;  //horizontal speed
		double dist = 181*1000;
		double ang = 58.3; // zero is vertical (as in landing) 58.3
		double aboveGroud = 13748; // meters above ground  2:25:40 (as in the simulation) 13748
		double time = 0;
		double dt = 1; // sec
		double acc=0; // Acceleration rate (m/s^2)
		double fuel = 121; //121
		double weight = WEIGHT_EMP + fuel;
		double NN = 0.7; // rate[0,1]


		double n1 = 0.04;
		double n2 = 0.001;
		double n3 = 0.1;
		double prevError = 0.0;
		double errorSum = 0.0;

		// ***** main simulation loop ******
		while(aboveGroud>=0) {
			if(time%10==0 || aboveGroud<100) {
				System.out.println("time: "+time+", vs:"+vs+", hs: "+hs+", dist: "+dist+", aboveGround: "+aboveGroud+", angle: "+ang+", weight: "+weight+", acc: "+acc +", fuel: "+fuel + ", NN: " +NN);
			}

			if((vs>2.5 || hs>2.5) && aboveGroud<5){
				System.out.println("crashed");
				break;
			}
			else if (vs<=2.5 && hs<=2.5 && aboveGroud<=5){
				System.out.println("landed");
				break;
			}

			double vsDest;
			// over 2 km above the ground
			if(aboveGroud>2000) {	// maintain a vertical speed of [20-25] m/s
				vsDest = 23;
			}
			// lower than 2 km - horizontal speed should be close to zero
			else {
				if(aboveGroud>125){
					vsDest = 15;
				}
				else{
					vsDest = 2.5;
				}

				if(ang>3) {
					ang-=3; // rotate to vertical position
				}
				else if(ang<-3){
					ang+=3;
				}
                else{
                    ang = 0;
                }

			}

			//PID
			double error = vsDest - vs; // if vs > vsDest -> error is negative -> we want bigger NN
			errorSum += error;
			double stopRate = error - prevError;
			double output = error*n1 + errorSum*n2 + stopRate*n3;
			prevError = error;
			NN-=output;


			// bound NN
			if (NN < 0) {
				NN = 0;
			}
			if (NN > 1) {
				NN = 1;
			}

			if(aboveGroud<=3){
				NN=0;
			}

			// main computations
			double ang_rad = Math.toRadians(ang);
			double h_acc = Math.sin(ang_rad)*acc;
			double v_acc = Math.cos(ang_rad)*acc;
			double vacc = Moon.getAcc(hs);
			time+=dt;
			double fuelPerSecond = dt*ALL_BURN*NN;
			if(fuel>0) {
				fuel -= fuelPerSecond;
				weight = WEIGHT_EMP + fuel;
				acc = NN* accMax(weight);
			}
			else { // ran out of fuel
				acc=0;
			}

			v_acc -= vacc;

			if(hs>0) {
				hs -= h_acc*dt;
				if(hs<0){
					hs = 0;
				}
			}

			dist -= hs*dt;
			vs -= v_acc*dt;
			aboveGroud -= dt*vs;
		}

	}
}
