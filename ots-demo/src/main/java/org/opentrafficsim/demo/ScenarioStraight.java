package org.opentrafficsim.demo;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.model.inputparameters.InputParameterException;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import nl.tudelft.simulation.language.DsolException;
import org.djunits.value.vdouble.scalar.*;
import org.djutils.draw.point.DirectedPoint2d;
import org.djutils.event.EventListener;
import org.djutils.io.URLResource;
import org.json.JSONArray;
import org.json.JSONObject;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.perception.HistoryManagerDevs;
import org.opentrafficsim.demo.ScenarioStraight.ScenarioStraightModel;
import org.opentrafficsim.draw.gtu.DefaultCarAnimation.GtuData.GtuMarker;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.DefaultLmrsPerceptionFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlanner;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlannerFactory;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.swing.gui.AnimationToggles;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import javax.naming.NamingException;
import java.awt.*;
import java.net.URI;
import java.net.URL;
import java.rmi.RemoteException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.djunits.unit.LengthUnit.METER;
import static org.djunits.unit.SpeedUnit.KM_PER_HOUR;

/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class ScenarioStraight extends OtsSimulationApplication<ScenarioStraightModel>
{
    /** */
    private static final long serialVersionUID = 20170407L;

    /** Simulation time. */
    public static final Time SIMTIME = Time.instantiateSI(3600);

    /**
     * Create a ScenarioStraight Swing application.
     * @param title the title of the Frame
     * @param panel the tabbed panel to display
     * @param model the model
     */
    public ScenarioStraight(final String title, final OtsAnimationPanel panel, final ScenarioStraightModel model)
    {
        super(model, panel, Map.of(DefaultsNl.TRUCK, GtuMarker.SQUARE));
    }

    @Override
    protected void setAnimationToggles()
    {
        AnimationToggles.setTextAnimationTogglesFull(getAnimationPanel());
        getAnimationPanel().getAnimationPanel().toggleClass(Link.class);
        getAnimationPanel().getAnimationPanel().toggleClass(Node.class);
        getAnimationPanel().getAnimationPanel().showClass(SpeedSign.class);
    }

    @Override
    protected void addTabs() {    }

    /**
     * Main program.
     * @param args the command line arguments (not used)
     */
    public static void main(final String[] args)
    {
        demo(true);
    }

    /**
     * Start the demo.
     * @param exitOnClose when running stand-alone: true; when running as part of a demo: false
     */
    public static void demo(final boolean exitOnClose)
    {
        try
        {
            OtsAnimator simulator = new OtsAnimator("ScenarioStraight");
            final ScenarioStraightModel otsModel = new ScenarioStraightModel(simulator);
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(SIMTIME.si), otsModel,
                    HistoryManagerDevs.noHistory(simulator));
            OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(), new Dimension(800, 600),
                    simulator, otsModel, DEFAULT_GTU_COLORERS, otsModel.getNetwork());
            ScenarioStraight app = new ScenarioStraight("ScenarioStraight", animationPanel, otsModel);
            app.setExitOnClose(exitOnClose);
            animationPanel.enableSimulationControlButtons();
        }
        catch (SimRuntimeException | NamingException | RemoteException | IndexOutOfBoundsException | DsolException exception)
        {
            exception.printStackTrace();
        }
    }


    public static class ScenarioStraightModel extends AbstractOtsModel implements EventListener, MyListener
    {
        /** */
        private static final long serialVersionUID = 20170407L;
        /** Number of cars created. */
        private int carsCreated = 0;
        /** The network. */
        private RoadNetwork network;
        /** The random number generator used to decide what kind of GTU to generate etc. */
        private StreamInterface stream = new MersenneTwister(12345);
        /** Strategical planner generator for cars. */
        private LaneBasedStrategicalPlannerFactory<?> strategicalPlannerGeneratorCars = null;
        private WebSocketClient client;

        private LaneBasedGtu egoGtu;
        private List<String> vehicleIdsAdded = new ArrayList<>();
        private String laneChange = "";

        /**
         * Constructor.
         * @param simulator the simulator
         */
        public ScenarioStraightModel(final OtsSimulatorInterface simulator)
        {
            super(simulator);
        }

        /**
         * Set network.
         * @param network set network.
         */
        public void setNetwork(final RoadNetwork network)
        {
            this.network = network;
        }

        @Override
        public void constructModel() throws SimRuntimeException
        {
            try
            {
                URI uri = new URI("ws://localhost:8099");
                client = new WebSocketClient(uri);
                client.setListener(this);

                this.strategicalPlannerGeneratorCars = new LaneBasedStrategicalRoutePlannerFactory(
                        new LmrsFactory(new IdmPlusFactory(this.stream), new DefaultLmrsPerceptionFactory()));
                URL xmlURL = URLResource.getResource("/resources/ScenarioStraight.xml");
                this.network = new RoadNetwork("ScenarioStraight", getSimulator());
                new XmlParser(this.network).setUrl(xmlURL).build();
                System.out.println("Network created");

                this.egoGtu = generateGTU(new Length(5, METER), "AB", 200);
                this.egoGtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);

//                gtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
//                generateGTU(new Length(1, METER), "cp1-lane1", 200);
//                generateGTU(new Length(1, METER), "cp4-lane1", 0);

                Thread getLocationThread = new Thread()
                {
                    @Override
                    public void run()
                    {
                        try {
//                            while (simulator.isStartingOrRunning()) {
                            while (true){
                                DirectedPoint2d position = egoGtu.getLocation();
                                double speed = egoGtu.getSpeed().getSI();
                                double acceleration = egoGtu.getAcceleration().getSI();
                                boolean isBrakingLightsOn = egoGtu.isBrakingLightsOn();
                                String turnIndicatorStatus = egoGtu.getTurnIndicatorStatus().name();
                                egoGtu.getLane();
                                JSONObject dataJson = new JSONObject();
                                dataJson.put("speed", speed);
                                dataJson.put("acceleration", acceleration);
                                dataJson.put("isBrakingLightsOn", isBrakingLightsOn);
                                dataJson.put("turnIndicatorStatus", turnIndicatorStatus);
                                dataJson.put("laneChange", laneChange);
                                laneChange = "";  // the request for lane change was send --> reset
                                JSONObject positionJson = new JSONObject();
                                positionJson.put("x", position.getX());
                                positionJson.put("y", position.getX());
                                dataJson.put("position", positionJson);

                                String msg = dataJson.toString();
                                client.sendMessage(msg);
                                try {
                                    Thread.sleep(100);
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }
                            }
                        } catch (Exception e) {
                            System.out.println("ERROR: " + e);
                        }
                    }
                };
                getLocationThread.start();
            }
            catch (Exception exception)
            {
                exception.printStackTrace();
            }
        }

        @Override
        public RoadNetwork getNetwork()
        {
            return this.network;
        }

        private Lane getLane(final CrossSectionLink link, final String id)
        {
            return (Lane) link.getCrossSectionElement(id);
        }

        protected final LaneBasedGtu generateGTU(Length initialPosition, String link_id, int maxSpeed)
                throws GtuException, NetworkException, SimRuntimeException, InputParameterException {
            if (maxSpeed < 0) {
                maxSpeed = 0;
            }

            // TODO: Implement parameter lane_id
            CrossSectionLink link = (CrossSectionLink) this.network.getLink(link_id);
            Lane lane = link.getLanes().get(1);
            // GTU itself
            Length vehicleLength = new Length(4, METER);
            LaneBasedGtu gtu = new LaneBasedGtu("" + (++this.carsCreated), DefaultsNl.CAR, vehicleLength, new Length(1.8, METER),
                    new Speed(maxSpeed, KM_PER_HOUR), vehicleLength.times(0.5), this.network);
            gtu.setNoLaneChangeDistance(Length.ZERO);
            gtu.setInstantaneousLaneChange(false);
            gtu.setMaximumAcceleration(Acceleration.instantiateSI(3.0));
            gtu.setMaximumDeceleration(Acceleration.instantiateSI(-8.0));

            // strategical planner
            LaneBasedStrategicalPlanner strategicalPlanner;
            Route route = null;
            Node start = this.network.getNode("l109-0");
            Node end = this.network.getNode("l52-1");
            strategicalPlanner = this.strategicalPlannerGeneratorCars.create(gtu, route, start, end);

            // init
            Speed initialSpeed = new Speed(0, KM_PER_HOUR);
            gtu.init(strategicalPlanner, new LanePosition(lane, initialPosition), initialSpeed);
            return gtu;
        }

        @Override
        public void onEvent(JSONObject data) {
            if (egoGtu != null && simulator.isStartingOrRunning()) {
                double egoX = data.getJSONObject("ego").getDouble("x");
                double egoY = data.getJSONObject("ego").getDouble("y");

                JSONArray objects = data.getJSONArray("objects");
                for (int i = 0; i < objects.length(); i++) {
                    JSONObject object = objects.getJSONObject(i);
                    String name = object.getString("name");
                    if (!name.contains("Vehicles") || name.equals("Vehicles.V600.Fiat500.main")) {  // there is another car
                        continue;
                    }
                    String id = object.getString("id");
                    if (vehicleIdsAdded.contains(id)) {
                        continue;
                    }
                    double vehicleX = object.getDouble("x");
                    double vehicleY = object.getDouble("y");
                    double vehicleV = object.getDouble("v");
                    double distance = Math.hypot(vehicleX - egoX, vehicleY - egoY);
                    try {
                        double absolutPosition = egoGtu.getReferencePosition().position().si;
                        String link_id = this.egoGtu.getLane().getLink().getId();
                        String lane_id = this.egoGtu.getLane().getId();
                        generateGTU(new Length(absolutPosition + distance, METER), link_id, (int) vehicleV);
                        vehicleIdsAdded.add(id);
                    } catch (GtuException | NetworkException | InputParameterException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }

        public void notify(final org.djutils.event.Event event) throws RemoteException
        {
            if (event.getType().equals(LaneBasedGtu.LANEBASED_MOVE_EVENT))
            {
                Object[] payload = (Object[]) event.getContent();
                String laneChangeDirection = (String) payload[5];  // this can either be "RIGHT" / "LEFT" / "NONE"
                if (!laneChangeDirection.equals("")) {
                    laneChange = laneChangeDirection;
                }
            }
        }
    }

}
