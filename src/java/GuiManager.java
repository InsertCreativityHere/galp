
package net.insertcreativity.galp;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.DisplayMode;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.HeadlessException;
import java.awt.event.KeyEvent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.SwingConstants;
import javax.swing.JToolBar;
import javax.swing.JTree;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.TreeNode;

public class GuiManager
{
    // The main application window.
    private final JFrame mainFrame;
    // The main menubar that's displayed at the top of the main application window. 
    private final JMenuBar menubar;
    // The main toolbar that's displayed under the menu bar.
    private final JToolBar toolbar;

    // The navigation pane, this component is used for displaying the sensors and their interfaces along with the
    // heirachy of all the sessions, experiments, and trials currently loaded into the program.
    private final JSplitPane navigationPane;
    // The sensor navigation pane, this displays the sensor interfaces and their sensors.
    private final JScrollPane sensorNavPane;
    // The data navigation pane, this displays the sessions, experiments, and trials currently in use by the program.
    private final JScrollPane dataNavPane;
    // The sensor tree, this contains the actual heirarchy of sensors and interfaces.
    private final JTree sensorTree;
    // The data tree, this contains the actual heirarchy of sessions, experiments, and trials.
    private final JTree dataTree;

    // The content panel, where the data spreadsheet and graphs are displayed.
    private final JPanel contentPanel;

    public GuiManager()
    {
        // Throw an exception if the system doesn't support rendering GUIs.
        if(GraphicsEnvironment.isHeadless())
        {
            throw new HeadlessException("System doesn't support graphics interfaces.");
        }

        // Get a refernece to the screens currently connected to the system.
        GraphicsDevice[] displays = GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices();
        // Throw an exception if there's no screens connected to the system.
        if(displays.length < 1)
        {
            throw new HeadlessException("No screens detected on the system.");
        }
        // Keep a reference to the main display and it's display settings.
        // This is the display the application is going to be opened on.
        GraphicsDevice mainDisplay = displays[0];
        DisplayMode mainDisplayMode = mainDisplay.getDisplayMode();

        // The GUI is built from the bottom up, starting with smaller components and then adding them to larger ones.



        //===== Create the data tree =====//
        DefaultMutableTreeNode dataRoot = new DefaultMutableTreeNode();
        dataTree = new JTree(dataRoot);
        dataTree.setRootVisible(false);
        dataTree.setEditable(true);

        //===== Create the sensor tree =====//
        DefaultMutableTreeNode sensorRoot = new DefaultMutableTreeNode();
        sensorTree = new JTree(sensorRoot);
        sensorTree.setRootVisible(false);
        sensorTree.setEditable(false);

        //===== Create the data nav pane for displaying sessions, experiments, and trials =====//
        dataNavPane = new JScrollPane(dataTree, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        dataNavPane.setWheelScrollingEnabled(true);

        //===== Create the sensor nav pane for displaying the sensors and their interfaces ====//
        sensorNavPane = new JScrollPane(sensorTree, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        sensorNavPane.setWheelScrollingEnabled(true);

        //===== Create a panel for displaying the data navigation =====//
        JPanel dataNavPanel = new JPanel(new BorderLayout());
        // Add a label at the top of the data nav panel.
        dataNavPanel.add(new JLabel("Data Explorer", SwingConstants.LEFT), BorderLayout.PAGE_START);
        // Add the data nav pane to the panel.
        dataNavPanel.add(dataNavPane, BorderLayout.CENTER);

        //===== Create a panel for displaying the sensor navigation =====//
        JPanel sensorNavPanel = new JPanel(new BorderLayout());
        // Add a label at the top of the sensor nav panel.
        sensorNavPanel.add(new JLabel("Sensors", SwingConstants.LEFT), BorderLayout.PAGE_START);
        // Add the sensor nav pane to the panel.
        sensorNavPanel.add(sensorNavPane, BorderLayout.CENTER);
        // Make the sensorNavPanel to be 150px high.
        sensorNavPanel.setPreferredSize(new Dimension(0, 150));

        //===== Create a split pane for holding the sensor and data navigation panes =====//
        navigationPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, true, sensorNavPanel, dataNavPanel);
        // Make the navigation pane 200px wide.
        navigationPane.setPreferredSize(new Dimension(200, 0));



        //===== Create a panel for displaying the data and graph panels =====//
        contentPanel = new JPanel();



        //===== Create the tool bar =====//
        toolbar = new JToolBar(JToolBar.HORIZONTAL);



        //===== Create the file menu =====//
        JMenu fileMenu = new JMenu("File");
        fileMenu.setMnemonic(KeyEvent.VK_F);

        //===== Create the edit menu =====//
        JMenu editMenu = new JMenu("Edit");
        editMenu.setMnemonic(KeyEvent.VK_E);

        //===== Create the view menu =====//
        JMenu viewMenu = new JMenu("View");
        viewMenu.setMnemonic(KeyEvent.VK_V);

        //===== Create the data menu =====//
        JMenu dataMenu = new JMenu("Data");
        dataMenu.setMnemonic(KeyEvent.VK_D);

        //===== Create the sensors menu =====//
        JMenu sensorsMenu = new JMenu("Sensors");
        sensorsMenu.setMnemonic(KeyEvent.VK_S);

        //===== Create the options menu =====//
        JMenu optionsMenu = new JMenu("Options");
        optionsMenu.setMnemonic(KeyEvent.VK_O);

        //===== Create the help menu =====//
        JMenu helpMenu = new JMenu("Help");
        helpMenu.setMnemonic(KeyEvent.VK_H);

        //===== Create the menu bar =====//
        menubar = new JMenuBar();
        // Add the menus to the menu bar.
        menubar.add(fileMenu);
        menubar.add(editMenu);
        menubar.add(viewMenu);
        menubar.add(dataMenu);
        menubar.add(sensorsMenu);
        menubar.add(optionsMenu);
        menubar.add(helpMenu);



        //===== Create a split pane for separating the navigation panel from the content panel =====//
        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, true, navigationPane, contentPanel);

        //===== Create a panel for placing the other panels and the toolbar into.
        JPanel mainPanel = new JPanel(new BorderLayout());
        // Add the toolbar at the top of the main panel.
        mainPanel.add(toolbar, BorderLayout.PAGE_START);
        // Add the application's panels into the main panel.
        mainPanel.add(splitPane, BorderLayout.CENTER);

        //===== Create the main application window =====//
        mainFrame = new JFrame("GALP (Goodbye Ancient Labmate Products)", mainDisplay.getDefaultConfiguration());
        // Add the menu bar to the main application window.
        mainFrame.setJMenuBar(menubar);
        // Set the main panel as the content of the application window.
        mainFrame.setContentPane(mainPanel);
        // Set that the frame shouldn't close when the user hits the close button. Instead we let the program decide
        // whether to close; so it can prompt the user about unsaved data, perform cleanup, etc...
        mainFrame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);

        // Pack and size all the windows components to get ready for displaying it.
        mainFrame.pack();
        // Make the application window 85% of the available screen space.
        mainFrame.setSize((int)(mainDisplayMode.getWidth() * 0.85), (int)(mainDisplayMode.getHeight() * 0.85));
    }

    public void displayGui()
    {
        // This centers the main application window on the screen.
        mainFrame.setLocationRelativeTo(null);
        // Display the main application window.
        mainFrame.setVisible(true);
    }
}
