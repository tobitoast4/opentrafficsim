package org.opentrafficsim.demo;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.model.inputparameters.InputParameterException;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import nl.tudelft.simulation.language.DsolException;
import org.djunits.unit.AccelerationUnit;
import org.djunits.value.vdouble.scalar.*;
import org.djutils.draw.point.DirectedPoint2d;
import org.djutils.event.EventListener;
import org.djutils.io.URLResource;
import org.json.JSONArray;
import org.json.JSONObject;
import org.opentrafficsim.base.parameters.ParameterTypes;
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
import java.util.LinkedHashMap;
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
        private int egoGtuMaxSpeed = 200;
        private Map<String, LaneBasedGtu> vehiclesMap = new LinkedHashMap<>();
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

                this.egoGtu = generateGTU(new DirectedPoint2d(10, 0, 0), egoGtuMaxSpeed, 1);
                this.egoGtu.addListener(this, LaneBasedGtu.LANEBASED_MOVE_EVENT);
                this.egoGtu.getParameters().setParameter(ParameterTypes.BCRIT,
                        new Acceleration(10, AccelerationUnit.METER_PER_SECOND_2));
                this.egoGtu.getParameters().setParameter(ParameterTypes.B,
                        new Acceleration(9, AccelerationUnit.METER_PER_SECOND_2));
                this.egoGtu.getParameters().setParameter(ParameterTypes.B0,
                    new Acceleration(6, AccelerationUnit.METER_PER_SECOND_2));

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
                                boolean isBrakingLightsOn = false;
                                try {
                                    isBrakingLightsOn = egoGtu.isBrakingLightsOn();
                                } catch (Exception e) {
                                    System.out.println(e);
                                }
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
                                positionJson.put("y", position.getY());
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

        protected final LaneBasedGtu generateGTU(DirectedPoint2d initialLocation, int maxSpeed, int laneID)
                throws GtuException, NetworkException, SimRuntimeException, InputParameterException {
            // TODO: Implement parameter lane_id
            CrossSectionLink link = (CrossSectionLink) this.network.getClosestLink(initialLocation);
            Lane lane = link.getLanes().get(laneID);
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
            Length length = new Length(link.lengthToClosestPoint(initialLocation), METER);
            gtu.init(strategicalPlanner, new LanePosition(lane, length), initialSpeed);
            return gtu;
        }

        @Override
        public void onEvent(JSONObject data) {
            if (egoGtu != null && simulator.isStartingOrRunning()) {
                double avX = data.getJSONObject("av").getDouble("x");
                double avY = data.getJSONObject("av").getDouble("y");

                JSONArray objects = data.getJSONArray("objects");
                for (int i = 0; i < objects.length(); i++) {
                    JSONObject object = objects.getJSONObject(i);
                    double objectX = object.getJSONObject("position").getDouble("x");
                    double objectY = object.getJSONObject("position").getDouble("y");
                    double objectV = object.getDouble("v");
                    int laneID = object.getInt("laneID");
                    double distance = Math.hypot(objectX - avX, objectY - avY);

                    String name = object.getString("name");
                    if (!name.contains("Vehicles") || name.equals("Vehicles.V600.Fiat500.main")) {  // there is another car
                        if (name.contains("sign.")) {
                            if (name.contains("274_")) {         // Speed limit  (doc/Referenz DatenbasisSCNX.pdf, p109)
                                String[] parts = name.split("_");
                                String lastPart = parts[1];
                                int speedLimit = Integer.parseInt(lastPart);
                                this.egoGtu.temporarySpeedLimit = new Speed(speedLimit, KM_PER_HOUR);
                            } else if (name.contains("278_")) {  // Speed limit removed
                                this.egoGtu.temporarySpeedLimit = null;
                            }
                        }
                    } else {
                        String id = object.getString("type");
                        if (vehiclesMap.containsKey(id)) {
                            continue;
//                            LaneBasedGtu gtu = vehiclesMap.get(id);
//                            this.network.removeGTU(gtu);
                        }
                        try {
                            if (laneID == 5) {
                                laneID = 1;
                            } else if (laneID == 4) {
                                laneID = 0;
                            } else {
                                throw new NetworkException("Invalid lane");
                            }

                            LaneBasedGtu newGtu = generateGTU(new DirectedPoint2d(objectX, objectY, 0), (int) objectV, laneID);
                            vehiclesMap.put(id, newGtu);
                        } catch (GtuException | NetworkException | InputParameterException e) {
                            throw new RuntimeException(e);
                        }
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
