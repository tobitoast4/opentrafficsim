package org.opentrafficsim.demo;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.dsol.model.inputparameters.InputParameterException;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import org.djunits.unit.DirectionUnit;
import org.djunits.unit.DurationUnit;
import org.djunits.unit.util.UNITS;
import org.djunits.value.vdouble.scalar.*;
import org.djutils.draw.point.DirectedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.traceverifier.TraceVerifier;
import org.json.JSONArray;
import org.json.JSONObject;
import org.opentrafficsim.base.parameters.Parameters;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.gtu.TurnIndicatorStatus;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.road.definitions.DefaultsRoadNl;
import org.opentrafficsim.road.gtu.lane.LaneBasedGtu;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.DefaultLmrsPerceptionFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.LmrsFactory;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlanner;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalPlannerFactory;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.LaneFactory;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.LaneType;

import java.io.IOException;
import java.net.URI;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

/**
 * Simulate traffic on a circular, two-lane road.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 */
public class MyRoadModel extends AbstractOtsModel implements UNITS, MyListener
{
    /** */
    private static final long serialVersionUID = 20141121L;

    /** Number of cars created. */
    private int carsCreated = 0;

    /** The probability that the next generated GTU is a passenger car. */
    private double carProbability;

    /** Minimum distance. */
    private Length minimumDistance = new Length(0, METER);

    /** The speed limit. */
    private Speed speedLimit = new Speed(100, KM_PER_HOUR);

    /** The sequence of Lanes that all vehicles will follow. */
    private List<List<Lane>> paths = new ArrayList<>();

    /** The random number generator used to decide what kind of GTU to generate etc. */
    private StreamInterface stream = new MersenneTwister(12345);

    /** Strategical planner generator for cars. */
    private LaneBasedStrategicalPlannerFactory<?> strategicalPlannerGeneratorCars = null;

    /** Strategical planner generator for trucks. */
    private LaneBasedStrategicalPlannerFactory<?> strategicalPlannerGeneratorTrucks = null;

    /** Car parameters. */
    private Parameters parametersCar;

    /** Truck parameters. */
    private Parameters parametersTruck;

    /** The RoadNetwork. */
    private final RoadNetwork network;
    private LaneBasedGtu egoGtu;
    private Lane[] lanes1;
    private Lane[] lanes2;
    private OtsSimulatorInterface simulator;
    private List<String> vehicleIdsAdded = new ArrayList<>();

    private double last_dirZ = 0;

    /**
     * Constructor.
     * @param simulator the simulator for this model
     */
    public MyRoadModel(final OtsSimulatorInterface simulator)
    {
        super(simulator);
        this.network = new RoadNetwork("network", simulator);
        this.simulator = simulator;
        MyRoadModel model = this;

        Thread getLocationThread = new Thread() {
            @Override
            public void run() {
                try {
                    URI uri = new URI("ws://localhost:8099");
                    WebSocketClient client = new WebSocketClient(uri);
                    client.setListener(model);
                    System.out.println("getLocationThread starts up");
                    while (true) {
                        if (egoGtu != null && simulator.isStartingOrRunning()) {
                            DirectedPoint2d location = egoGtu.getLocation();
                            double dirZ = egoGtu.getDirZ();
                            double dirZ_diff = round(dirZ - last_dirZ, 3);
                            if (dirZ_diff != 0) {
                                Instant timestamp = Instant.now();
                                System.out.println("Current timestamp: " + timestamp);
                                System.out.println(dirZ_diff);
                            }
                            last_dirZ = dirZ;

                            boolean brakeLightsOn = egoGtu.isBrakingLightsOn();
                            TurnIndicatorStatus turnIndicatorStatus = egoGtu.getTurnIndicatorStatus();
                            double speed = egoGtu.getSpeed().getSI();
                            double acceleration = egoGtu.getAcceleration().getSI();
                            Lane lane = egoGtu.getLane();


                            JSONObject gtuData = new JSONObject();
                            gtuData.put("acceleration", acceleration);
                            gtuData.put("speed", speed);
                            gtuData.put("brakeLightsOn", brakeLightsOn);
                            gtuData.put("turnIndicatorStatus", turnIndicatorStatus);
                            JSONObject gtuLocation = new JSONObject();
                            gtuLocation.put("x", location.getX());
                            gtuLocation.put("y", location.getY());
                            gtuData.put("location", gtuLocation);

                            String msg = gtuData.toString();

                            client.sendMessage(msg);
                        }
                        try {
                            Thread.sleep(1);
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

    private double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        long factor = (long) Math.pow(10, places);
        value = value * factor;
        long tmp = Math.round(value);
        return (double) tmp / factor;
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
                if (!name.contains("Vehicles")) {  // there is another car
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
                    generateGTU(new Length(absolutPosition + distance, METER), lanes1[1], DefaultsNl.CAR, (int) vehicleV);
                    vehicleIdsAdded.add(id);
                } catch (GtuException | NetworkException | InputParameterException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    /**
     * Returns path.
     * @param index the rank number of the path
     * @return the set of lanes for the specified index
     */
    public List<Lane> getPath(final int index)
    {
        return this.paths.get(index);
    }

    /**
     * Sample the state of the simulation.
     * @param tv sampler or verifier of the state
     */
    public void sample(final TraceVerifier tv)
    {
        try
        {
            StringBuilder state = new StringBuilder();
            for (Gtu gtu : this.network.getGTUs())
            {
                LaneBasedGtu lbg = (LaneBasedGtu) gtu;
                state.append(String.format("%s: %130.130s ", lbg.getId(), lbg.getLocation().toString()));
            }

            tv.sample(this.simulator.getSimulatorTime().toString(), state.toString());
            this.simulator.scheduleEventRel(new Duration(1, DurationUnit.SECOND), this, "sample", new Object[] {tv});
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    @Override
    public void constructModel() throws SimRuntimeException
    {
        try
        {
            // TraceVerifier tv = new TraceVerifier("C:/Temp/circularRoadTrace.txt");
            // this.simulator.scheduleEventRel(new Duration(1, DurationUnit.SECOND), this, this, "sample", new Object[] { tv });
            // TraceVerifier tv = new TraceVerifier("C:/Temp/circularRoadTraceEndState.txt");
            // this.simulator.scheduleEventRel(new Duration(3599.99, DurationUnit.SECOND), this, this, "sample",
            // new Object[] { tv });
            final int laneCount = 2;
            for (int laneIndex = 0; laneIndex < laneCount; laneIndex++)
            {
                this.paths.add(new ArrayList<Lane>());
            }

            this.carProbability = (double) 0.8;
            double radius = 100;
            double headway = 1000.0 / 30;
            double headwayVariability = 0;

//            this.parametersCar = InputParameterHelper.getParametersCar(getInputParameterMap());
//            this.parametersTruck = InputParameterHelper.getParametersTruck(getInputParameterMap());

            this.strategicalPlannerGeneratorCars = new LaneBasedStrategicalRoutePlannerFactory(
                    new LmrsFactory(new IdmPlusFactory(this.stream), new DefaultLmrsPerceptionFactory()));
            this.strategicalPlannerGeneratorTrucks = new LaneBasedStrategicalRoutePlannerFactory(
                    new LmrsFactory(new IdmPlusFactory(this.stream), new DefaultLmrsPerceptionFactory()));

            GtuType gtuType = DefaultsNl.CAR;
            LaneType laneType = DefaultsRoadNl.TWO_WAY_LANE;
            Node start = new Node(this.network, "Start", new Point2d(radius, 0), new Direction(90, DirectionUnit.EAST_DEGREE));
            Node halfway =
                    new Node(this.network, "Halfway", new Point2d(-radius, 0), new Direction(270, DirectionUnit.EAST_DEGREE));

            Point2d[] coordsHalf1 = new Point2d[127];
            for (int i = 0; i < coordsHalf1.length; i++) {
                double angle = Math.PI * i / (coordsHalf1.length - 1);
                coordsHalf1[i] = new Point2d(radius * Math.cos(angle), radius * Math.sin(angle));
            }
            lanes1 = LaneFactory.makeMultiLane(this.network, "FirstHalf", start, halfway, coordsHalf1, laneCount,
                    laneType, this.speedLimit, this.simulator, DefaultsNl.VEHICLE);
            Point2d[] coordsHalf2 = new Point2d[127];
            for (int i = 0; i < coordsHalf2.length; i++) {
                double angle = Math.PI + Math.PI * i / (coordsHalf2.length - 1);
                coordsHalf2[i] = new Point2d(radius * Math.cos(angle), radius * Math.sin(angle));
            }
            lanes2 = LaneFactory.makeMultiLane(this.network, "SecondHalf", halfway, start, coordsHalf2, laneCount,
                    laneType, this.speedLimit, this.simulator, DefaultsNl.VEHICLE);
            for (int laneIndex = 0; laneIndex < laneCount; laneIndex++) {
                this.paths.get(laneIndex).add(lanes1[laneIndex]);
                this.paths.get(laneIndex).add(lanes2[laneIndex]);
            }

            this.egoGtu = generateGTU(new Length(10, METER), lanes1[1], gtuType, 200);
            generateGTU(new Length(100, METER), lanes1[1], gtuType, 0);
            // Put the (not very evenly spaced) cars on the track
        }
        catch (Exception exception)
        {
            exception.printStackTrace();
        }
    }

    protected final LaneBasedGtu generateGTU(final Length initialPosition, final Lane lane, final GtuType gtuType, int maxSpeed)
            throws GtuException, NetworkException, SimRuntimeException, InputParameterException
    {
        // GTU itself
        boolean generateTruck = false;
        Length vehicleLength = new Length(4, METER);
        LaneBasedGtu gtu = new LaneBasedGtu("" + (++this.carsCreated), gtuType, vehicleLength, new Length(1.8, METER),
                new Speed(maxSpeed, KM_PER_HOUR), vehicleLength.times(0.5), this.network);
//        gtu.setParameters(generateTruck ? this.parametersTruck : this.parametersCar);
        gtu.setNoLaneChangeDistance(Length.ZERO);
        gtu.setInstantaneousLaneChange(false);
        gtu.setMaximumAcceleration(Acceleration.instantiateSI(3.0));
        gtu.setMaximumDeceleration(Acceleration.instantiateSI(-8.0));

        // strategical planner
        LaneBasedStrategicalPlanner strategicalPlanner;
        Route route = null;
        strategicalPlanner = this.strategicalPlannerGeneratorCars.create(gtu, route, null, null);

        // init
        Speed initialSpeed = new Speed(0, KM_PER_HOUR);
        gtu.init(strategicalPlanner, new LanePosition(lane, initialPosition), initialSpeed);
        return gtu;
    }

    @Override
    public RoadNetwork getNetwork()
    {
        return this.network;
    }

    /**
     * Returns the minimum distance.
     * @return minimumDistance
     */
    public final Length getMinimumDistance()
    {
        return this.minimumDistance;
    }

    /**
     * Stop simulation and throw an Error.
     * @param theSimulator the simulator
     * @param errorMessage the error message
     */
    public void stopSimulator(final OtsSimulatorInterface theSimulator, final String errorMessage)
    {
        System.out.println("Error: " + errorMessage);
        try
        {
            if (theSimulator.isStartingOrRunning())
            {
                theSimulator.stop();
            }
        }
        catch (SimRuntimeException exception)
        {
            exception.printStackTrace();
        }
        throw new Error(errorMessage);
    }
}
