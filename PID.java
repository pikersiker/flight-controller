public class PID {
    private final double kp;
    private final double ki;
    private final double kd;

    private double lastError = 0;
    private double integral = 0;

    private long lastTime = System.nanoTime();

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double calculate(double expected, double actual) {
        // assigns current time at method call to calculate change in time for later calls
        long currentTime = System.nanoTime();
        // divides by a billion to convert nanoseconds to seconds
        double deltaTime = (double) (currentTime - lastTime) / 1_000_000_000;
        // calculates error
        double error = expected - actual;
        // to prevent integral windup
        if (error < 0.01) {
            reset();
            // if error is small enough, doesn't use pid
            return 0;
        }
        // updates integral every time method is called
        integral += error * deltaTime;
        // calculates change in error over time
        double derivative = (error - lastError) / deltaTime;
        // calculates output
        double output = kp * error + ki * integral + kd * derivative;
        // updates lastError every time method is called
        lastError = error;
        // updates last time to calculate change in time
        lastTime = currentTime;
        return output;
    }
    // implement later for pid tuning
    public void reset() {
        lastError = 0;
        integral = 0;
        lastTime = System.nanoTime();
    }
}
