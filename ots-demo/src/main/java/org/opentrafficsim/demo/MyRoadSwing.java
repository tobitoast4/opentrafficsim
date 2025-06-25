package org.opentrafficsim.demo;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.swing.gui.inputparameters.TabbedParameterDialog;
import nl.tudelft.simulation.language.DsolException;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Time;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulator;
import org.opentrafficsim.core.perception.HistoryManagerDevs;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import javax.naming.NamingException;
import javax.swing.*;
import java.awt.*;
import java.rmi.RemoteException;

/**
 * Circular road simulation demo.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 */
public class MyRoadSwing extends OtsSimulationApplication<MyRoadModel>
{
    /** */
    private static final long serialVersionUID = 1L;

    /**
     * Create a CircularRoad Swing application.
     * @param title the title of the Frame
     * @param panel the tabbed panel to display
     * @param model the model
     */
    public MyRoadSwing(final String title, final OtsAnimationPanel panel, final MyRoadModel model)
    {
        super(model, panel, DefaultsFactory.GTU_TYPE_MARKERS.toMap());

        // NetworkAnimation networkAnimation = new NetworkAnimation(model.getNetwork());
        // networkAnimation.addDrawingInfoClass(Lane.class, new DrawingInfoShape<>(Color.GRAY));
        RoadNetwork network = model.getNetwork();
        System.out.println(network.getLinkMap());
    }

    /**
     * Main program.
     * @param args the command line arguments (not used)
     */
    public static void main(final String[] args)
    {
//         simulatorDemo();
        demo(true);
    }

    /**
     * Run the simulation without animation.
     */
    public static void simulatorDemo()
    {
        try
        {
            OtsSimulator simulator = new OtsSimulator("MyRoadSwing");
            final MyRoadModel otsModel = new MyRoadModel(simulator);

            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(3600.0), otsModel,
                    HistoryManagerDevs.noHistory(simulator));
//            Thread getLocationThread = new Thread()
//            {
//                @Override
//                public void run()
//                {
//                    try {
//                        URI uri = new URI("ws://localhost:8099");
//                        WebSocketClient client = new WebSocketClient(uri);
//                        System.out.println("getLocationThread starts up");
//                        int iteration = 0;
//                        int getLocationCalls = 0;
//                        while (simulator.isStartingOrRunning())
//                        {
//                            iteration++;
//                            for (Gtu gtu : otsModel.getNetwork().getGTUs())
//                            {
//                                gtu.getLocation();
//                                getLocationCalls++;
//                                String msg = """
//                                        { "acceleration": 1.0, "steeringWheel": 0.2 }
//                                    """;
//                                client.sendMessage(msg);
//                            }
//                            try
//                            {
//                                Thread.sleep(1);
//                            }
//                            catch (InterruptedException e)
//                            {
//                                e.printStackTrace();
//                            }
//                        }
//                        System.out.println("getLocationThread exits after " + iteration + " iterations and " + getLocationCalls
//                                + " getLocation calls");
//                    } catch (Exception e) {
//                        System.out.println("ERROR: " + e);
//                    }
//
//                }
//
//            };
            simulator.start();
//            getLocationThread.start();
            while (simulator.isStartingOrRunning())
            {
                Thread.sleep(1000);
                // System.out.println("Simulator time is " + simulator.getSimulatorTime());
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        System.exit(0);
    }

    /**
     * Find the start simulation button and click it.
     * @param component some component that could be the start button, or a container that contains the start button
     * @return true if the start button was found (and clicked); false otherwise
     */
    public static boolean clickStart(final Component component)
    {
        if (component instanceof JButton)
        {
            JButton button = (JButton) component;
            if (button.getText().contains("Start simulation model"))
            {
                button.doClick();
                System.out.println("Auto clicked the start button");
                return true;
            }
        }
        else if (component instanceof Container)
        {
            for (Component comp : ((Container) component).getComponents())
            {
                if (clickStart(comp))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Click the button that starts the animated simulation.
     * @param component some component that (hopefully) is, or contains the start button
     * @return true if the button was found (and clicked); false if the start button was not found
     */
    public static boolean clickRunPause(final Component component)
    {
        if (component instanceof JButton)
        {
            JButton button = (JButton) component;
            // System.out.println("Found button with name " + button.getName());
            if (button.getName().equals("runPauseButton"))
            {
                button.doClick();
                System.out.println("Auto clicked the run button");
                return true;
            }
        }
        else if (component instanceof Container)
        {
            for (Component comp : ((Container) component).getComponents())
            {
                if (clickRunPause(comp))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Start the demo.
     * @param exitOnClose when running stand-alone: true; when running as part of a demo: false
     */
    public static void demo(final boolean exitOnClose)
    {
        try
        {
            OtsAnimator simulator = new OtsAnimator("MyRoadSwing");
            final MyRoadModel otsModel = new MyRoadModel(simulator);
            if (TabbedParameterDialog.process(otsModel.getInputParameterMap()))
            {
                simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(3600.0), otsModel,
                        HistoryManagerDevs.noHistory(simulator));
                OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(),
                        new Dimension(800, 600), simulator, otsModel, DEFAULT_GTU_COLORERS, otsModel.getNetwork());
                MyRoadSwing app = new MyRoadSwing("My Road", animationPanel, otsModel);
                app.setExitOnClose(exitOnClose);
                animationPanel.enableSimulationControlButtons();
            }
            else
            {
                if (exitOnClose)
                {
                    System.exit(0);
                }
            }
        }
        catch (SimRuntimeException | NamingException | RemoteException | DsolException exception)
        {
            exception.printStackTrace();
        }
    }
}
