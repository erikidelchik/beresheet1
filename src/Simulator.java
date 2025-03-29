import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;

public class Simulator extends JPanel implements Runnable {
    private Beresheet_101 beresheet;
    private List<Point.Double> trajectory;
    // Variables for dynamic zooming (observed maximum values)
    private double observedMaxX = 0;
    private double observedMaxAlt = 13748; // initial altitude

    public Simulator() {
        setPreferredSize(new Dimension(800, 600));
        beresheet = new Beresheet_101();
        trajectory = new ArrayList<>();
        new Thread(this).start();
    }

    @Override
    public void run() {
        while (beresheet.getAboveGround() > 0) {
            beresheet.simulateStep();
            // Compute simulation x coordinate: 0 at start (using expected full distance)
            double simX = 181 * 1000 - beresheet.getDist();
            if (simX > observedMaxX) observedMaxX = simX;
            if (beresheet.getAboveGround() > observedMaxAlt) observedMaxAlt = beresheet.getAboveGround();
            trajectory.add(new Point.Double(simX, beresheet.getAboveGround()));
            repaint();
            try {
                Thread.sleep(5); // control animation speed
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        // Fill background.
        g.setColor(Color.BLACK);
        g.fillRect(0, 0, getWidth(), getHeight());

        // Dynamic scaling (with a margin)
        double marginFactor = 1.1;
        double xScale = getWidth() / (observedMaxX * marginFactor);
        double yScale = getHeight() / (observedMaxAlt * marginFactor);

        // Draw trajectory.
        g.setColor(Color.GREEN);
        for (int i = 1; i < trajectory.size(); i++) {
            double prevX = trajectory.get(i - 1).x;
            double prevAlt = trajectory.get(i - 1).y;
            double curX = trajectory.get(i).x;
            double curAlt = trajectory.get(i).y;
            int x1 = (int) (prevX * xScale);
            int y1 = getHeight() - (int) (prevAlt * yScale);
            int x2 = (int) (curX * xScale);
            int y2 = getHeight() - (int) (curAlt * yScale);
            g.drawLine(x1, y1, x2, y2);
        }

        // Draw current position as a rotated rectangle.
        // Compute simulation x coordinate.
        double simX = 181 * 1000 - beresheet.getDist();
        int centerX = (int) (simX * xScale);
        int centerY = getHeight() - (int) (beresheet.getAboveGround() * yScale);

        Graphics2D g2d = (Graphics2D) g;
        // Save current transform.
        AffineTransform oldTransform = g2d.getTransform();
        // Translate to the rectangle's center.
        g2d.translate(centerX, centerY);
        // Rotate by the current angle (assuming getAng() returns degrees, where 0 is vertical).
        g2d.rotate(Math.toRadians(beresheet.getAng()));
        // Draw the rectangle (centered at (0,0); adjust width and height as desired).
        g2d.setColor(Color.RED);
        // For example, a rectangle 10px wide and 20px high (vertical when angle == 0)
        g2d.fillRect(-5, -10, 10, 20);
        // Restore original transform.
        g2d.setTransform(oldTransform);

        // Display simulation info.
        g.setColor(Color.WHITE);
        g.drawString(String.format("Time: %.0f s", beresheet.getTime()), 10, 20);
        g.drawString(String.format("Altitude: %.2f m", beresheet.getAboveGround()), 10, 40);
        g.drawString(String.format("angle: %.2f", beresheet.getAng()), 10, 60);
        g.drawString(String.format("Vertical Speed: %.2f m/s", beresheet.getVs()), 10, 80);
        g.drawString(String.format("Horizontal Speed: %.2f m/s", beresheet.getHs()), 10, 100);
        g.drawString(String.format("Fuel: %.2f liters", beresheet.getFuel()), 10, 120);
    }

    public static void main(String[] args) {
        JFrame frame = new JFrame("Beresheet Simulator");
        Simulator simPanel = new Simulator();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(simPanel);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}
