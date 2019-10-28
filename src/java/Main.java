
package net.insertcreativity.galp;

public class Main
{
    public static final DataManager dataManager = new DataManager();

    public static void main(String[] args)
    {
        GuiManager gui = new GuiManager();
        gui.displayGui();
    }

    public static void handleException(String type, Exception exception)
    {
        System.err.println("Error encountered: " + exception.getMessage());
        System.err.println("type = " + type);
        exception.printStackTrace();
    }
}
