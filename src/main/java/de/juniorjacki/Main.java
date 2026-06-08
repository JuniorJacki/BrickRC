package de.juniorjacki;

import de.juniorjacki.remotebrick.DataSubscriber;
import de.juniorjacki.remotebrick.Hub;
import de.juniorjacki.remotebrick.devices.Motor;
import de.juniorjacki.remotebrick.devices.UltrasonicSensor;
import de.juniorjacki.remotebrick.types.Image;
import de.juniorjacki.remotebrick.types.PathDirection;
import de.juniorjacki.remotebrick.types.Port;
import de.juniorjacki.remotebrick.types.StopType;
import de.juniorjacki.utils.Logger;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Main {
    public static void main(String[] args) {

        Hub robot = null;
        Hub controller = null;
        int i = 0;
        while (robot == null || controller == null) {
            if(robot == null) robot = Hub.connect("A8:E2:C1:9C:52:E1");
            if(controller == null) controller = Hub.connect("A8:E2:C1:9B:9B:C3");
            Logger.info("Connection attempt " + i);
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {}
        }
        Logger.info("Connected Successfully");

        startNormalization(controller);
        startPowerControl(controller, robot);
        startTurnControl(controller, robot);
        startDistanceReport(controller, robot);
    }

    static AtomicBoolean updateDistance = new AtomicBoolean(false);
    static AtomicInteger lastDist = new AtomicInteger(0);

    static void startDistanceReport(Hub controller,Hub robot) {
        DataSubscriber.registerSimpleListener((UltrasonicSensor)robot.getDevice(Port.F), UltrasonicSensor.UltrasonicSensorDataType.Distance, newValue -> {
            if ((Integer) newValue != lastDist.get()) {
                updateDistance.set(true);
                lastDist.set((Integer) newValue);
            }
        });

        new Thread(()-> {
            while (true) {
                if (updateDistance.get()) {
                    updateDistance.set(false);
                    int distance = ((UltrasonicSensor) robot.getDevice(Port.F)).getDistance();
                    if (distance > 200) {
                        robot.getControl().display().text("N").sendAsync();
                        controller.getControl().display().text("N").sendAsync();
                    } else {
                        Image image = fromNumber(distance);
                        robot.getControl().display().image(image).sendAsync();
                        controller.getControl().display().image(image).sendAsync();
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {
                    }
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
        }).start();
    }

    static AtomicInteger lastPow = new AtomicInteger(0);
    static AtomicBoolean updatePow = new AtomicBoolean(false);

    static void startPowerControl(Hub controller,Hub robot) {
        Motor powerControl = (Motor)controller.getDevice(Port.B);
        Motor leftPower = (Motor)robot.getDevice(Port.C);
        Motor rightPower = (Motor)robot.getDevice(Port.E);
        DataSubscriber.registerSimpleListener(powerControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
            int power = (Integer) newValue+45;
            if(power < 5 && power > -5) {
                power = 0;
            } else {
                power *= -2;
                power = Math.min(100,power);
                power = Math.max(-100,power);
            }
            if (power != lastPow.get()) {
                lastPow.set(power);
                updatePow.set(true);
            }
        });
        new Thread(()-> {
            while (true) {
                if (updatePow.get()) {
                    updatePow.set(false);
                    int lastPower = lastPow.get();
                    if (lastPower == 0) {
                        robot.getControl().move().stop(leftPower,rightPower,StopType.BRAKE).send();
                    } else {
                        robot.getControl().move().startPowers(leftPower,rightPower,lastPower,lastPower,100).send();
                    }
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {
                    }
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
        }).start();
    }

    static AtomicInteger lastTurn = new AtomicInteger(0);
    static AtomicBoolean updateTurn = new AtomicBoolean(true);

    static void startTurnControl(Hub controller,Hub robot) {
        Motor turnControl = (Motor)controller.getDevice(Port.E);
        Motor turnPower = (Motor)robot.getDevice(Port.A);
        DataSubscriber.registerSimpleListener(turnControl, Motor.MotorDataType.AbsolutePosition,newValue -> {
            int turn = (Integer) newValue;
            if(turn < 5 && turn > -5) {
                turn = 0;
            } else {
                turn *= 2;
                turn = Math.min(120,turn);
                turn = Math.max(-120,turn);
            }
            if (turn != lastTurn.get()) {
                lastTurn.set(turn);
                updateTurn.set(true);
            }
        });
        new Thread(()->{
            while(true) {
                if (updateTurn.get()) {
                    updateTurn.set(false);
                    Logger.info("Turn Control " + lastTurn.get());
                    turnPower.getControl().goToRelativePosition(lastTurn.get(),40,false,StopType.BRAKE,100,100).send();
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException ignored) {}
                }
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {}
            }
        }).start();
    }


    static AtomicBoolean tryNormalizePow = new AtomicBoolean(false);
    static AtomicBoolean tryNormalizeTurn = new AtomicBoolean(false);
    static final int moveDetect = 1;

    static void startNormalization(Hub controller) {
        Motor powerControl = (Motor)controller.getDevice(Port.B);
        Motor turnControl = (Motor)controller.getDevice(Port.E);
        normalize(tryNormalizePow,powerControl,-45);
        //normalize(tryNormalizeTurn,turnControl,0);
    }

    static void normalize(AtomicBoolean activator, Motor motor,int defaultPos) {
        DataSubscriber.registerSimpleListener(motor, Motor.MotorDataType.AbsolutePosition,newValue -> {
            activator.set(true);
        });
        new Thread(() -> {
            while (true) {
                if(activator.get()) {
                    int pos = motor.getPosition();
                    CompletableFuture<String> result = motor.getControl().goToPositionWithDirection(defaultPos,15,PathDirection.SHORTEST,false,StopType.COAST,10,10).sendAsync();
                    while (true) {
                        try {
                            Thread.sleep(350);
                        } catch (InterruptedException ignored) {}
                        if (result.isDone()) {
                            activator.set(false);
                            break;
                        }
                        if (motor.getPosition() <= pos - moveDetect || motor.getPosition() >= pos + moveDetect) {
                            Logger.info("Continue");
                            pos = motor.getPosition();
                            continue;
                        } else {
                            Logger.info("Break");
                            motor.getControl().stop(StopType.BRAKE,100).send();
                            break;
                        }
                    }
                }
                try {
                    Thread.sleep(200);
                } catch (Exception ignored) {}
            }
        }).start();
    }



    /**
     * Generiert ein Image-Objekt, das eine Zahl von 0 bis 200 anzeigt.
     * * @param number Die anzuzeigende Zahl (0-200).
     * @return Ein neues Image-Objekt mit der dargestellten Zahl.
     * @throws IllegalArgumentException wenn die Zahl außerhalb von 0-200 liegt.
     */
    public static Image fromNumber(int number) {
        if (number < 0 || number > 200) {
            throw new IllegalArgumentException("Zahl muss zwischen 0 und 200 liegen!");
        }

        Image img = new Image();

        final int[][][] FONT = {
                {{1,1}, {1,0}, {1,0}, {1,0}, {1,1}}, // 0
                {{0,1}, {0,1}, {0,1}, {0,1}, {0,1}}, // 1
                {{1,1}, {0,1}, {1,1}, {1,0}, {1,1}}, // 2
                {{1,1}, {0,1}, {1,1}, {0,1}, {1,1}}, // 3
                {{1,0}, {1,0}, {1,1}, {0,1}, {0,1}}, // 4
                {{1,1}, {1,0}, {1,1}, {0,1}, {1,1}}, // 5
                {{1,1}, {1,0}, {1,1}, {1,0}, {1,1}}, // 6
                {{1,1}, {0,1}, {0,1}, {0,1}, {0,1}}, // 7
                {{1,1}, {1,1}, {1,1}, {1,1}, {1,1}}, // 8
                {{1,1}, {1,1}, {1,1}, {0,1}, {1,1}}  // 9
        };

        if (number < 10) {
            int digit = number;
            drawDigit(img, FONT[digit], 1, 9);
        }
        else if (number < 100) {
            int tens = number / 10;
            int ones = number % 10;
            drawDigit(img, FONT[tens], 0, 9);
            drawDigit(img, FONT[ones], 3, 7);
        }
        else {
            int hundreds = number / 100;
            int tens = (number / 10) % 10;
            int ones = number % 10;
            for (int y = 0; y < 5; y++) {
                img.setPixel(0, y, 9);
                if (hundreds == 2) {
                    img.setPixel(1, y, 9);
                }
            }
            drawDigit(img, FONT[tens], hundreds == 1 ? 2 : 3, 6);
            drawDigit(img, FONT[ones], 4, 3);
        }

        return img;
    }


    private static void drawDigit(Image img, int[][] digitPattern, int startX, int brightness) {
        for (int y = 0; y < 5; y++) {
            for (int x = 0; x < digitPattern[y].length; x++) {
                if (startX + x > 4) continue;
                if (digitPattern[y][x] == 1) {
                    img.setPixel(startX + x, y, brightness);
                }
            }
        }
    }
}