
package net.insertcreativity.galp;

public class GuiManager
{
    private JFrame mainFrame;
    private JTreeNode navigationRoot;
    private JTree navigationTree;
    private JPanel navigationPanel;

    public JFrame createMainGui()
    {
        // Create the applications main window.
        mainFrame = new JFrame();

        // Load the nagivation tree data
        ... do what the comment says
        // Create the navigation tree
        navigationTree = new JTree(navigationRoot);

        // Create a scrollable panel to display the navigation tree in.
        JScrollPane navigationScrollPane = new JScrollPane(navigationTree, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCOLLBAR_AS_NEEDED);
        navigationScrollPane.setWheelScrollingEnabled(true);
        // Create a panel for encapsulating all the navigation UI.
        navigationPanel = new JPanel(new BorderLayout(), true);
        navigationPanel.add(navigationScrollPane, BorderLayout.CENTER);



        JSplitPane contentPane = new JSplitPane()
        JPanel dataPanel = new JPanel();
    }
}
