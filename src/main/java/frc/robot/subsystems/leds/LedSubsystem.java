package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import javax.imageio.ImageIO;

import java.io.File;
import java.io.IOException;
import java.awt.image.BufferedImage;



public class LedSubsystem extends SubsystemBase {
    //Row length
    public static final int LED_WIDTH=1;
    //Collum length
    public static final int LED_LENGTH=19;
    public static final int LED_SIZE = LED_WIDTH*LED_LENGTH;
    
    int[] RGB = new int[LED_SIZE * 3];
    BufferedImage hello;
    BufferedImage there;

    int ticks;
    int currentPort = 5;
    

    AddressableLED leds;
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_SIZE);

    public LedSubsystem() {
        
        try {
            leds = new AddressableLED(Constants.LED_PORT);
            leds.setLength(LED_SIZE);
            leds.start();
            // hello = createImageIcon("hello.png");
            // there = createImageIcon("there.png");
        } 
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void DisplayImage(BufferedImage image) {
        for (int xPos = 0; xPos < 32; xPos++) {
            for (int yPos = 0; yPos < 8; yPos++) {
                int pixelColor = image.getRGB(xPos, yPos);
                RGB[(yPos * 32 + xPos) * 3] = (pixelColor & 0x00ff0000) >> 16;
                RGB[(yPos * 32 + xPos) * 3 + 1] = (pixelColor & 0x0000ff00) >> 8;
                RGB[(yPos * 32 + xPos) * 3 + 2] = (pixelColor & 0x000000ff);
            }
        }

        leds.setLength(LED_SIZE);

        for (int i = 0; i < LED_SIZE * 3; i += 3) {
            ledBuffer.setRGB(i / 3, RGB[i], RGB[i + 1], RGB[i + 2]);
        }

        leds.setData(ledBuffer);
        
    }

    private static BufferedImage createImageIcon(String name) {
        File imgDir = new File(Filesystem.getDeployDirectory(), "images");
        File imgFile = new File(imgDir, name);
        if (imgFile.exists()) {
            try {
                return ImageIO.read(imgFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            System.err.println("*** Couldn't find file: " + imgFile.getAbsolutePath());
        }
        return null;
    }

    public void LED_SetFromTextFile() {

        leds.setLength(LED_SIZE);

        for (int i = 0; i < LED_SIZE * 3; i += 3) {
            ledBuffer.setRGB(i, RGB[i], RGB[i + 1], RGB[i + 2]);
            leds.setData(ledBuffer);
        }

    }

    public void turnOff() {
        SetAllColor(0,0,0);
        //leds.close(); Comment out just in case 
    }

    public void SetAllColor(int r, int g, int b) {
        for (int i = 0; i < LED_SIZE; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    public void SetRowColor(int row,int r, int g, int b) {
        int start = row * LED_WIDTH;
        for (int i = 0; i < LED_WIDTH; ++i) {
            ledBuffer.setRGB(start + i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    public void StartColor() {
        SetAllColor(30, 245, 30);
    }

    // public void Periodic(int databoardPort) {
    //     if (databoardPort != currentPort) {
    //         currentPort = databoardPort;
    //         leds = new AddressableLED(currentPort);
    //     }
    //     ticks++;
    //     if (ticks < 250){
    //         StartColor();
    //     } else if (ticks < 500) {
    //         DisplayImage(hello);
    //     } else {
    //         DisplayImage(there);
    //     }
    //     if (ticks > 750) {
    //         ticks = 250;
    //     }
    // }
    //Indicator of having a Note and the LEDs beung that indidcator
    // public void NoteIndicator () {
    //     if (Robot.getMap().shooter.isPieceInUpperIntake()==true)
    //     {
    //         SetAllColor(250, 90, 0);
    //     }
    //     else
    //     {
    //         SetAllColor(0,0,0);
    //     }
    // }

    public void setLEDs(Color[] colors) 
    {
        for(int i = 0; i < ledBuffer.getLength(); i++) 
        {
            ledBuffer.setRGB(i, (int)colors[i].red, (int)colors[i].green, (int)colors[i].blue);
        }
        leds.setData(ledBuffer);
    }

    public int getNumberOfLEDs() 
    {
        return LED_SIZE;
    }
    public void setRGB(int i,int r, int b,int g)
    {
       ledBuffer.setRGB(i,r,g,b);    
    }
    
    public boolean AreLEDsDoingStuff()
    {
        boolean Truth = false;
        //int c=0;
        for(int i=0; i<RGB.length; i+=3)
        {
            if(RGB[i]==0 && RGB[i+1]==0 && RGB [i+2]==0 && Truth!=true) //(c!=2||Truth!=true))
            {
                Truth=false;
                //c=1;
            }
            else
            {
                Truth=true;
                //c=2;
            }
        }   
        return Truth;
    }

    public void binaryLed(int number) {
        // this works only if the LED bank has enough spots to represent the binary
        int mask = 0x01;
        int index = 0;
        while (number > 0) {
            int bit = mask << index;
            if ((number & bit) != 0) {
                ledBuffer.setRGB(index, 250, 10, 0);
            } else {
                ledBuffer.setRGB(index, 0, 0, 0);
            }
            ++index;
            number = number & ~bit;
        }
        leds.setData(ledBuffer);
    }
}


