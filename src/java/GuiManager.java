
package net.insertcreativity.galp;

import java.awt.DisplayMode;
import java.awt.GraphicsEnvironment;
import java.awt.HeadlessException;
import java.awt.KeyEvent;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JToolBar;
import javax.swing.JTree;
import javax.swing.JTreeNode;

public class GuiManager
{
    // The main application window.
    private final JFrame mainFrame;
    // The main menubar that's displayed at the top of the main application window. 
    private final JMenuBar menubar;
    // The main toolbar that's displayed under the menu bar.
    private final JToolBar toolbar;

    // The navigation panel, this component is used for displaying the sessions, experiments, and trials currently
    // loaded into or that are running in the program.
    private final JPanel navigationPanel;

    // The content panel, where the data spreadsheet and graphs are displayed.
    private final JPanel contentPanel;




    // The Root node that all sessions, experiments, and trials are stored under in the navigation panel.
    private final JTreeNode navigationRoot;
    // The actual tree that displays the sessions, experiments, and trials in the navigation panel.
    private final JTree navigationTree;

    // The data panel, this component is used to display the raw numerical data in a spreadsheet like fashion.
    private final JPanel dataPanel;

    // The graph panel, this component is used for displaying graphs of various data from the program.
    private final JPanel graphPanel;


    private final void createMainGui()
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
        // Keep a reference to the main display for launching the application on.
        GraphicsDevice mainDisplay = displays[0];
        DisplayMode mainDisplayMode = mainDisplay.


        // The GUI is built from the bottom up, starting with smaller components and then adding them to larger ones.

        //===== Create the navigation tree =====//
        navigationTree = null;//TODO

        //===== Create the navigation panel =====//
        navigationScrollPane = new JScrollPane(navigationTree, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCOLLBAR_AS_NEEDED);
        // Enable mouse scrolling of the navigation pane.
        navigationScrollPane.setWheelScrollingEnabled(true);



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

        //===== Create the analyze menu =====//
        JMenu analyzeMenu = new JMenu("Analyze");
        analyzeMenu.setMnemonic(KeyEvent.VK_A);

        //===== Create the help menu =====//
        JMenu helpMenu = new JMenu("Help");
        helpMenu.setMnemonic(KeyEvent.VK_H);

        //===== Create the menu bar =====//
        menubar = new JMenuBar();
        // Add the menus to the menu bar.
        menubar.add(fileMenu);
        menubar.add(editMenu);
        menubar.add(viewMenu);
        menubar.add(analyzeMenu);
        menubar.add(helpMenu);



        //===== Create a split pane for separating the navigation panel from the content panel =====//
        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, true, navigationPanel, contentPanel);

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

        //===== Display the main application window =====//
        // Pack and size all the windows components to get ready for displaying it.
        mainFrame.pack();
        // Get the size of the screen the application window is going to be made on.
        // Make the application window 75% of the screen space.
        mainFrame.setSize((int)(mainDisplayMode.getWidth * 0.75), (int)(mainDisplayMode * 0.75));
        // This centers the main application window on the screen.
        mainFrame.setLocationRelativeTo(null);
        // Display the main application window.
        mainFrame.setVisible(true);
    }

    public static void main(String[] args)
    {
        createMainGui();
    }
}
