package com.mmkrusniak.frame;

import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.DroneTeam;
import com.mmkrusniak.drone.cooperative.*;

import javax.swing.*;
import javax.swing.border.BevelBorder;
import javax.swing.text.html.HTMLDocument;
import java.awt.*;
import java.awt.event.*;
import java.io.File;
import java.lang.reflect.InvocationTargetException;

import static com.mmkrusniak.frame.GUIUtil.*;

public class GUIMain {

    private static Area area;
    private static DroneTeam drones;
    private static JPanel coveragePanel;
    private static JTabbedPane dataPanel;

    // The area type changes automatically when the relevant options are changed, so it's not itself an option but
    // rather is stored here. We could probably accomplish the same thing with option depends too.
    private static String areaType = "Polygon";

    // Listener to kill the thread when the frame's gone
    private static final WindowListener END_ON_CLOSE = new WindowListener() {
        public void windowOpened(WindowEvent windowEvent) {}
        public void windowClosing(WindowEvent windowEvent) { System.exit(0); }
        public void windowClosed(WindowEvent windowEvent) {}
        public void windowIconified(WindowEvent windowEvent) {}
        public void windowDeiconified(WindowEvent windowEvent) {}
        public void windowActivated(WindowEvent windowEvent) {}
        public void windowDeactivated(WindowEvent windowEvent) {}
    };

    // listener to collect the mouse location and repaint on a click
    private static final MouseListener RENDER_ON_CLICK = new MouseListener() {
        public void mouseClicked(MouseEvent e) {
            Input.mouse = guiToArea(e.getX(), e.getY());
            Input.mouseRaw = new java.awt.Point(e.getX(), e.getY());
            Option.set("input", "mouse_in_area", Input.mouse);
            SwingUtilities.getRoot(coveragePanel).setCursor(Cursor.getDefaultCursor());
            coveragePanel.repaint(); }
        public void mousePressed(MouseEvent mouseEvent) {}
        public void mouseReleased(MouseEvent mouseEvent) {}
        public void mouseEntered(MouseEvent mouseEvent) {}
        public void mouseExited(MouseEvent mouseEvent) {}
    };

    private static final MouseMotionListener TRACK_MOUSE_MOTION = new MouseMotionListener() {
        public void mouseDragged(MouseEvent e) {}
        public void mouseMoved(MouseEvent e) {
            Input.mouse = guiToArea(e.getX(), e.getY());
            Input.mouseRaw = new java.awt.Point(e.getX(), e.getY());
        }
    };

    // listener to repaint the panel as the drones advance
    private static final DroneTeam.DroneTeamListener REPAINT_ON_TRAVERSAL = new DroneTeam.DroneTeamListener() {
        public void onTraversalFinished() { coveragePanel.repaint(); remakeDataPanel(); }
        public void onTraversalProgressed() {
            if(Option.get("graphics", "render", Boolean.class)) coveragePanel.repaint();
            if(Option.get("graphics", "live_metrics", Boolean.class)) remakeDataPanel();
        }
    };

    public static void main(String[] args) {
        makeOptions();

        area = new Area(5);
        drones = new DroneTeam(getAllowedDroneSet(0)[0]);
        drones.addListener(REPAINT_ON_TRAVERSAL);

        JFrame frame = new JFrame("Coverage Path Visualizer");
        frame.setExtendedState(frame.getExtendedState() | JFrame.MAXIMIZED_BOTH);
        frame.setLayout(new GridBagLayout());
        frame.addWindowListener(END_ON_CLOSE);

        try { UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception e) { e.printStackTrace(); }

        fillFrame(frame);
        frame.setVisible(true);
        frame.setMinimumSize(new Dimension(800, 400));
        runDrones();
    }

    /**
     * Registers all the options that the GUI displays and uses.
     */
    public static void makeOptions() {
        Global.defaultOptions(getAllowedDroneSet(0));
        Option.register("input", "mouse_in_area", com.mmkrusniak.geom.Point.class, "Last click location (internal)",
                new com.mmkrusniak.geom.Point(0, 0));

        Option.listenAll("area", e -> {area = getNewArea(); runDrones();});
        Option.listen("area", "n", e -> areaType = "Polygon");
        Option.listen("area", "file", e -> areaType = "File");
        Option.listenAll("objects", e -> {area.redistribute(); runDrones();});
        Option.listenAll("drone", e -> { drones = getNewDroneTeam(); runDrones(); remakeDataPanel();});
        Option.listenAll("drone_team", e -> { drones = getNewDroneTeam(); runDrones(); remakeDataPanel();});
        Option.listenAll("physics", e -> runDrones());
        Option.listenAll("graphics", e -> coveragePanel.repaint());
        Option.listenAll("dist", e -> { area.recreateDistribution(); runDrones(); });
    }

    /**
     * Fills a JFrame with the details of the application.
     * @param frame A JFrame.
     */
    public static void fillFrame(JFrame frame) {
        frame.addWindowListener(END_ON_CLOSE);
        GridBagConstraints gbc = new GridBagConstraints();
        Insets insets = new Insets(5, 5, 5, 5);
        gbc.gridx = 2;
        gbc.gridy = 0;
        gbc.gridwidth = 1;
        gbc.gridheight = 2;
        gbc.weightx = 0.15;
        gbc.weighty = 1.0;
        gbc.anchor = GridBagConstraints.WEST;
        gbc.insets = insets;
        gbc.fill = GridBagConstraints.BOTH;

        dataPanel = new JTabbedPane();
        remakeDataPanel();
        frame.add(dataPanel, gbc);

        gbc.gridx = 1;
        gbc.weightx = 1;
        coveragePanel = new CoveragePanel();
        coveragePanel.setBorder(new BevelBorder(BevelBorder.LOWERED));
        coveragePanel.addMouseListener(RENDER_ON_CLICK);
        coveragePanel.addMouseMotionListener(TRACK_MOUSE_MOTION);
        frame.add(coveragePanel, gbc);

        // Tabs
        gbc.gridx = 0;
        gbc.gridheight = 1;
        gbc.weightx = 0.1;
        JPanel sidebar = new JPanel(new BorderLayout());
        sidebar.add(new JLabel("Path Coverage Simulator"), BorderLayout.PAGE_START);
        frame.add(sidebar, gbc);

        JTabbedPane tabs = new JTabbedPane();
        sidebar.add(tabs, BorderLayout.CENTER);

        JPanel areaPanel = new JPanel();
        areaPanel.setLayout(new BoxLayout(areaPanel, BoxLayout.PAGE_AXIS));
        areaPanel.add(Box.createRigidArea(new Dimension(2, 10)));
        areaPanel.add(centeredLabel("Generate random polygons:"));
        areaPanel.add(Option.getPanel("area", "n"));
        areaPanel.add(Option.getPanel("area", "size"));
        areaPanel.add(Option.getPanel("area", "jaggedness"));
        areaPanel.add(horizontalLine());
        areaPanel.add(centeredLabel("Import an area from a file:"));
        areaPanel.add(Option.getPanel("area", "file"));
        areaPanel.add(fillerPanel());
        tabs.add(areaPanel, "Area");

        JPanel objectsPanel = new JPanel();
        objectsPanel.setLayout(new BoxLayout(objectsPanel, BoxLayout.PAGE_AXIS));
        objectsPanel.add(Option.getPanel("objects", "n"));
        objectsPanel.add(Option.getPanel("dist", "spread"));
        objectsPanel.add(Option.getPanel("dist", "modes"));
        objectsPanel.add(Option.getPanel("dist", "mode"));
        objectsPanel.add(fillerPanel());
        tabs.add(objectsPanel, "Objects");

        JScrollPane dronePanel = new JScrollPane();
        dronePanel.setViewportView(remakeDronePanel());
        dronePanel.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
        dronePanel.setPreferredSize(dronePanel.getMaximumSize());
        ActionListener updateDrones = e -> {
            dronePanel.setViewportView(remakeDronePanel());
            dronePanel.repaint();
        } ;
        Option.listen("drone", "n", updateDrones);
        Option.listenAll("drone_team", updateDrones);
        JPanel droneTab = new JPanel();
        droneTab.setLayout(new BoxLayout(droneTab, BoxLayout.PAGE_AXIS));
        droneTab.add(Option.getPanel("drone", "n"));
        droneTab.add(Option.getPanel("drone", "dest"));
        droneTab.add(Option.getPanel("drone", "home"));
        droneTab.add(dronePanel);
        tabs.add(droneTab, "Drone");

        frame.setVisible(true);
        coveragePanel.repaint();

        JPanel graphicsPanel = new JPanel();
        graphicsPanel.setLayout(new BoxLayout(graphicsPanel, BoxLayout.PAGE_AXIS));
        graphicsPanel.add(Option.getPanel("graphics", "render"));
        graphicsPanel.add(Option.getPanel("graphics", "live_metrics"));
        graphicsPanel.add(Option.getPanel("graphics", "trace"));
        graphicsPanel.add(Option.getPanel("graphics", "broadcast_indicators"));
        graphicsPanel.add(Option.getPanel("graphics", "pdf"));
        graphicsPanel.add(Option.getPanel("graphics", "captures"));
        graphicsPanel.add(fillerPanel());
        tabs.add(graphicsPanel, "Graphics");

        JPanel otherPanel = new JPanel();
        otherPanel.setLayout(new BoxLayout(otherPanel, BoxLayout.PAGE_AXIS));
        otherPanel.add(Option.getPanel("detection", "threshold"));
        otherPanel.add(Option.getPanel("physics", "drag"));
        otherPanel.add(Option.getPanel("physics", "kp"));
        otherPanel.add(Option.getPanel("physics", "ki"));
        otherPanel.add(Option.getPanel("physics", "kd"));
        otherPanel.add(Option.getPanel("physics", "sim_speed"));
        otherPanel.add(Option.getPanel("dist", "modes"));
        otherPanel.add(fillerPanel());
        tabs.add(otherPanel, "Other");
    }


    /**
     * A JPanel which displays the route of the currently selected drones on the area.
     */
    static class CoveragePanel extends JPanel {

        // animates the little spinning wheel during traversal :P
        private int animTick = 0;

        /**
         * Renders the panel. It's called automatically by Swing. It draws the area and the path of all drones currently
         * in the team.
         * @param g A Java graphics canvas.
         */
        @Override
        public void paintComponent(Graphics g) {
            super.paintComponent(g);
            g.setFont(new Font("Helvetica", Font.PLAIN, 18));
            if(area == null) {
                g.setColor(Color.DARK_GRAY);
                g.drawString("Select a region", 5, 22);
                return;
            }
            Graphics2D g2d = (Graphics2D) g;

            double scale = Math.max(area.getWidth(), area.getHeight()) / Math.min(getWidth(), getHeight());
            g2d.scale(1/scale, 1/scale);
            g2d.translate(area.getHull().leftmost().ix(), area.getHull().topmost().iy());
            area.render(g2d);

            drones.render(g2d);

            g2d.setColor(Color.DARK_GRAY);
            g2d.scale(scale, scale);
            if(drones.isTraversing()) {
                g2d.setStroke(new BasicStroke(2));
                g2d.drawArc(10, 10, 8, 8, (int) (animTick/64.0*360), 120);
            }
            g2d.setStroke(new BasicStroke(1));
            g2d.drawString(area.getName(), 25, 20);
            animTick = (animTick + 1) % 64;
        }
    }

    /**
     * Runs the selected traversal algorithms.
     */
    public static void runDrones() {
        drones.traverse(area, null);
    }

    /**
     * Gives a new area based on the current options. If the area is random, a totally new random area is generated; if
     * it is from a file, only the distribution of objects will change.
     * @return A new area based on the current options.
     */
    public static Area getNewArea() {
        Area result = area;
        switch (areaType) {
            case "Polygon": result = new Area(Option.get("area", "n", Integer.class)); break;
            case "File": result = new Area(Option.get("area", "file", File.class)); break;
        }
        return result;
    }

    /**
     * Recreates the drone panel to include options for each member of the drone team.
     * @return A JPanel allowing the user to select some options about drones.
     */
    public static JPanel remakeDronePanel() {
        JPanel result = new JPanel();
        result.setLayout(new BoxLayout(result, BoxLayout.PAGE_AXIS));
        int n = Option.get("drone", "n", Integer.class);
        int height = 0;
        for (int i = 0; i < n; i++) {
            JPanel panel = Option.getPanel("drone_team", String.valueOf(i));
            result.add(panel);
            height += panel.getMinimumSize().height;
        }
        result.add(fillerPanel());
        result.setPreferredSize(new Dimension(result.getWidth(), height));
        return result;
    }

    /**
     * Generates a DroneTeam from the current options for number and type of drones.
     * @return A DroneTeam based on the current options.
     */
    public static DroneTeam getNewDroneTeam() {
        drones.stop();
        int n = Option.get("drone", "n", Integer.class);
        Drone[] team = new Drone[n];
        for (int i = 0; i < n; i++) {
            if(! Option.exists("drone_team", String.valueOf(i)))
                Option.register("drone_team", String.valueOf(i), Drone.class,
                        "Drone #" + (i+1), getAllowedDroneSet(i));
            team[i] = Option.get("drone_team", String.valueOf(i), Drone.class);
        }
        DroneTeam result = new DroneTeam(team);
        result.addListener(REPAINT_ON_TRAVERSAL);
        return result;
    }

    /**
     * Gives an array of new Drones (all of the same ID) of various types from which the user can select.
     * @param id The ID of the new drones.
     * @return An array of various Drones with that ID.
     */
    public static Drone[] getAllowedDroneSet(int id) {
        return new Drone[]{
                new ConvexOptimDrone(area, id),
                new PSODrone(area, id),
                new MapElitesDrone(area, id),
        };
    }

    /**
     * Scales a point on the coverage panel to its equivalent point in the currently active area.
     * @param guiX X location of the point within the coverage panel.
     * @param guiY Y location of the point within the coverage panel.
     * @return A point scaled to the units of the area that the coverage panel shows.
     */
    public static com.mmkrusniak.geom.Point guiToArea(int guiX, int guiY) {
        double scale = Math.max(area.getWidth(), area.getHeight()) / Math.min(coveragePanel.getWidth(),
                coveragePanel.getHeight());

        return new com.mmkrusniak.geom.Point(guiX * scale, guiY * scale);
    }

    public static void remakeDataPanel() {
        try {
            SwingUtilities.invokeAndWait(() -> {
                HTMLDocument[] docs = drones.dronesToHTML();
                JTextPane pane;
                for (int i = 0; i < drones.size(); i++) {
                    if (i < dataPanel.getTabCount() && dataPanel.getComponentAt(i) != null)
                        pane = (JTextPane) dataPanel.getComponentAt(i);
                    else {
                        pane = new JTextPane();
                        pane.setEditable(false);
                        pane.setContentType("text/html");
                        dataPanel.add(pane, "Drone " + i);
                    }
                    pane.setStyledDocument(docs[i]);
                    pane.repaint();
                }
                for(int i = drones.size(); i < dataPanel.getTabCount(); i++) dataPanel.remove(i);
            });
        } catch (InterruptedException | InvocationTargetException e) {
            e.printStackTrace();
        }
    }
}
