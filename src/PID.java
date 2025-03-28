public class PID {
    double n1;
    double n2;
    double n3;
    double prevError = 0.0;
    double errorSum = 0.0;
    double error;
    double stopRate;

    public PID(double n1, double n2, double n3){
        this.n1 = n1;
        this.n2 = n2;
        this.n3 = n3;
    }

    public double calculateNNoutput(double vsDest,double vs){
        error = vsDest - vs; // if vs > vsDest -> error is negative -> we want bigger NN
        errorSum += error;
        stopRate = error - prevError;
        prevError = error;
        return error*n1 + errorSum*n2 + stopRate*n3;
    }
}
