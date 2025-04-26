public class Main {
    public static void main(String[] args) {
        // sets PID constants
        PID pid = new PID(1, 0.1, 0.5);
        // sets desired angles, future implementation will allow for flight path correction
        double expectedX = 0;
        double expectedY = 0;
        int waitTime = 50;

        boolean running = true;
        while (running) {
            double outputX = pid.calculate(expectedX, Position.getX());
            double outputY = pid.calculate(expectedY, Position.getY());
            System.out.println(outputX + ", " + outputY); // for debugging
            // waits to save system resources
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            // add implementation to control servo using outputX and outputY
        }
    }
}
