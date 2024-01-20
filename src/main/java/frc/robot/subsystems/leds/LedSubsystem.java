package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.imageio.ImageIO;

import java.io.File;
import java.io.IOException;
import java.awt.image.BufferedImage;



public class LedSubsystem extends SubsystemBase {
    public static final int LED_WIDTH=8;
    public static final int LED_LENGTH=32;
    public static final int LED_SIZE = LED_WIDTH*LED_LENGTH;
    int[] RGB = new int[LED_SIZE * 3];
    BufferedImage hello;
    BufferedImage there;

    int ticks;
    int currentPort = 0;

    AddressableLED leds;
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_SIZE);

    public LedSubsystem() {
        
        try {
            
            leds = new AddressableLED((int)SmartDashboard.getNumber("LED Port", 0));
            leds.setLength(LED_SIZE);
            leds.start();
            hello = createImageIcon("hello.png");
            there = createImageIcon("there.png");
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public void DisplayImage(BufferedImage image) {
        for (int xPos = 0; xPos < 32; xPos++) {
            for (int yPos = 0; yPos < 8; yPos++) {
                int pixelColor = image.getRGB((int) xPos, (int) yPos);
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
        leds.start();
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
            leds.start();
        }

    }

    public void turnOff() {
        leds.close();
    }

    public void SetAllColor(int r, int g, int b) {
        for (int i = 0; i < LED_SIZE; i++) {
            ledBuffer.setRGB(i, r, g, b);
            leds.setData(ledBuffer);
            leds.start();
        }
    }

    public void StartColor() {
        SetAllColor(30, 245, 30);
    }

    public void Periodic(int databoardPort) {
        if (databoardPort != currentPort) {
            currentPort = databoardPort;
            leds = new AddressableLED(currentPort);
        }
        ticks++;
        if (ticks < 250){
            StartColor();
        } else if (ticks < 500) {
            DisplayImage(hello);
        } else {
            DisplayImage(there);
        }
        if (ticks > 750) {
            ticks = 250;
        }
    }

    public void Touch(String object) {
        if (object.equals("thing")) {
            SetAllColor(20, 30, 250);
        }
    }

    public void ChargeUpSeq()
    {

        for (int row = 0; row < LED_SIZE/8; row++) 
        {
            for(int i=0; i< 8; i++)
            {
                ledBuffer.setRGB(i,250,50,50);
            }
            
        }
        leds.setData(ledBuffer);
            leds.start();
    }
}
