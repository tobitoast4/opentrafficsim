package org.opentrafficsim.demo;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import nl.tudelft.simulation.jstats.distributions.DistNormal;
import nl.tudelft.simulation.jstats.distributions.DistUniform;
import nl.tudelft.simulation.jstats.streams.MersenneTwister;
import nl.tudelft.simulation.jstats.streams.StreamInterface;
import nl.tudelft.simulation.language.DsolException;
import org.djunits.unit.*;
import org.djunits.value.vdouble.scalar.*;
import org.djutils.io.URLResource;
import org.opentrafficsim.animation.GraphLaneUtil;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.base.parameters.ParameterSet;
import org.opentrafficsim.base.parameters.ParameterTypes;
import org.opentrafficsim.core.definitions.DefaultsNl;
import org.opentrafficsim.core.distributions.ConstantSupplier;
import org.opentrafficsim.core.distributions.FrequencyAndObject;
import org.opentrafficsim.core.distributions.ObjectDistribution;
import org.opentrafficsim.core.dsol.AbstractOtsModel;
import org.opentrafficsim.core.dsol.OtsAnimator;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.GtuException;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.idgenerator.IdSupplier;
import org.opentrafficsim.core.network.Link;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.core.network.route.ProbabilisticRouteGenerator;
import org.opentrafficsim.core.network.route.Route;
import org.opentrafficsim.core.parameters.ParameterFactory;
import org.opentrafficsim.core.parameters.ParameterFactoryByType;
import org.opentrafficsim.core.perception.HistoryManagerDevs;
import org.opentrafficsim.core.units.distributions.ContinuousDistDoubleScalar;
import org.opentrafficsim.demo.MyNetworkDemo.MyNetworkDemoModel;
import org.opentrafficsim.draw.graphs.GraphPath;
import org.opentrafficsim.draw.graphs.PlotScheduler;
import org.opentrafficsim.draw.graphs.TrajectoryPlot;
import org.opentrafficsim.draw.gtu.DefaultCarAnimation.GtuData.GtuMarker;
import org.opentrafficsim.road.gtu.generator.GeneratorPositions;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator;
import org.opentrafficsim.road.gtu.generator.LaneBasedGtuGenerator.RoomChecker;
import org.opentrafficsim.road.gtu.generator.TtcRoomChecker;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplate;
import org.opentrafficsim.road.gtu.generator.characteristics.LaneBasedGtuTemplateDistribution;
import org.opentrafficsim.road.gtu.generator.headway.HeadwayGenerator;
import org.opentrafficsim.road.gtu.lane.tactical.LaneBasedTacticalPlannerFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.AbstractIdm;
import org.opentrafficsim.road.gtu.lane.tactical.following.CarFollowingModelFactory;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlus;
import org.opentrafficsim.road.gtu.lane.tactical.following.IdmPlusFactory;
import org.opentrafficsim.road.gtu.lane.tactical.lmrs.*;
import org.opentrafficsim.road.gtu.lane.tactical.util.lmrs.*;
import org.opentrafficsim.road.gtu.strategical.LaneBasedStrategicalRoutePlannerFactory;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.network.lane.CrossSectionLink;
import org.opentrafficsim.road.network.lane.Lane;
import org.opentrafficsim.road.network.lane.LanePosition;
import org.opentrafficsim.road.network.lane.object.SpeedSign;
import org.opentrafficsim.road.network.sampling.LaneDataRoad;
import org.opentrafficsim.road.network.sampling.RoadSampler;
import org.opentrafficsim.swing.graphs.OtsPlotScheduler;
import org.opentrafficsim.swing.graphs.SwingPlot;
import org.opentrafficsim.swing.graphs.SwingTrajectoryPlot;
import org.opentrafficsim.swing.gui.AnimationToggles;
import org.opentrafficsim.swing.gui.OtsAnimationPanel;
import org.opentrafficsim.swing.gui.OtsSimulationApplication;

import javax.naming.NamingException;
import java.awt.*;
import java.net.URL;
import java.rmi.RemoteException;
import java.util.*;
import java.util.List;
import java.util.function.Supplier;

/**
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 * @author <a href="https://github.com/wjschakel">Wouter Schakel</a>
 */
public class MyNetworkDemo extends OtsSimulationApplication<MyNetworkDemoModel>
{
    /** */
    private static final long serialVersionUID = 20170407L;

    /** Simulation time. */
    public static final Time SIMTIME = Time.instantiateSI(3600);

    /**
     * Create a MyNetworkDemo Swing application.
     * @param title the title of the Frame
     * @param panel the tabbed panel to display
     * @param model the model
     */
    public MyNetworkDemo(final String title, final OtsAnimationPanel panel, final MyNetworkDemoModel model)
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
    protected void addTabs()
    {
        GraphPath<LaneDataRoad> path;
//        try
//        {
//            Lane start = ((CrossSectionLink) getModel().getNetwork().getLink("AB")).getLanes().get(1);
//            path = GraphLaneUtil.createPath("Right lane", start);
//        }
//        catch (NetworkException exception)
//        {
//            throw new RuntimeException("Could not create a path as a lane has no set speed limit.", exception);
//        }
//        RoadSampler sampler = new RoadSampler(getModel().getNetwork());
//        GraphPath.initRecording(sampler, path);
//        PlotScheduler scheduler = new OtsPlotScheduler(getModel().getSimulator());
//        Duration updateInterval = Duration.instantiateSI(10.0);
//        SwingPlot plot = new SwingTrajectoryPlot(
//                new TrajectoryPlot("Trajectory right lane", updateInterval, scheduler, sampler.getSamplerData(), path));
//        getAnimationPanel().getTabbedPane().addTab(getAnimationPanel().getTabbedPane().getTabCount(), "Trajectories",
//                plot.getContentPane());
    }

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
            OtsAnimator simulator = new OtsAnimator("MyNetworkDemo");
            final MyNetworkDemoModel otsModel = new MyNetworkDemoModel(simulator);
            simulator.initialize(Time.ZERO, Duration.ZERO, Duration.instantiateSI(SIMTIME.si), otsModel,
                    HistoryManagerDevs.noHistory(simulator));
            OtsAnimationPanel animationPanel = new OtsAnimationPanel(otsModel.getNetwork().getExtent(), new Dimension(800, 600),
                    simulator, otsModel, DEFAULT_GTU_COLORERS, otsModel.getNetwork());
            MyNetworkDemo app = new MyNetworkDemo("MyNetworkDemo", animationPanel, otsModel);
            app.setExitOnClose(exitOnClose);
            animationPanel.enableSimulationControlButtons();
        }
        catch (SimRuntimeException | NamingException | RemoteException | IndexOutOfBoundsException | DsolException exception)
        {
            exception.printStackTrace();
        }
    }


    public static class MyNetworkDemoModel extends AbstractOtsModel
    {
        /** */
        private static final long serialVersionUID = 20170407L;

        /** The network. */
        private RoadNetwork network;

        /**
         * Constructor.
         * @param simulator the simulator
         */
        public MyNetworkDemoModel(final OtsSimulatorInterface simulator)
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
                URL xmlURL = URLResource.getResource("/resources/DemoNetwork.xml");
                this.network = new RoadNetwork("MyNetworkDemo", getSimulator());
                new XmlParser(this.network).setUrl(xmlURL).build();
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
    }

}
