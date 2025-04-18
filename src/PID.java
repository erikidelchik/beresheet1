public class PID{
    double n1;
    double n2;
    double n3;
    double hs;
    double vs;
    double prevError = 0.0;
    double errorSum = 0.0;
    double NNerror;
    double angError;
    double angPrevError = 0.0;
    double angErrorSum = 0.0;
    double angStopRate;
    double stopRate;
    double dt;

    public PID(double n1, double n2, double n3, double dt){
        this.n1 = n1;
        this.n2 = n2;
        this.n3 = n3;
        this.dt = dt;
    }

    public double calculateNNoutput(double vsDest,double vs, double dt){
        NNerror = vsDest - vs; // if vs > vsDest -> error is negative -> we want bigger NN
        errorSum += n1*NNerror*dt;
        stopRate = (NNerror - prevError)/dt;
        prevError = NNerror;
        return NNerror*n1 + errorSum*n2 + stopRate*n3;
    }

    public double calculateAngleOutput(double angleDest,double angle,double dt){
        angError = angleDest - angle;
        angErrorSum += 0.08*angError*dt;
        angStopRate = (angError-angPrevError)/dt;
        angPrevError = angError;
        return angError*(0.04) + angErrorSum*(0.00001) + angStopRate*(0.1);
    }

}
