//TODO I should make there be a plugin like system, where it scans a directory or extra sensor packages can be loaded in at runtime instead of required they're programmed into this repository and packaged into this jar too...
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
