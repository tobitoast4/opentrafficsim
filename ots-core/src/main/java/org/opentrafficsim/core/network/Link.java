package org.opentrafficsim.core.network;

import java.io.Serializable;
import java.util.LinkedHashSet;
import java.util.Set;

import org.djunits.value.vdouble.scalar.Length;
import org.djutils.base.Identifiable;
import org.djutils.draw.bounds.Bounds2d;
import org.djutils.draw.function.ContinuousPiecewiseLinearFunction;
import org.djutils.draw.line.PolyLine2d;
import org.djutils.draw.line.Polygon2d;
import org.djutils.draw.point.DirectedPoint2d;
import org.djutils.draw.point.Point2d;
import org.djutils.event.EventType;
import org.djutils.event.LocalEventProducer;
import org.djutils.exceptions.Throw;
import org.djutils.metadata.MetaData;
import org.djutils.metadata.ObjectDescriptor;
import org.opentrafficsim.base.HierarchicallyTyped;
import org.opentrafficsim.base.geometry.OtsLine2d;
import org.opentrafficsim.base.geometry.OtsShape;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.Gtu;

/**
 * A standard implementation of a link between two Nodes.
 * <p>
 * Copyright (c) 2013-2024 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * <p>
 * $LastChangedDate$, @version $Revision$, by $Author$, initial version Aug 19, 2014 <br>
 * @author <a href="https://github.com/averbraeck">Alexander Verbraeck</a>
 * @author <a href="https://github.com/peter-knoppers">Peter Knoppers</a>
 */
public class Link extends LocalEventProducer
        implements HierarchicallyTyped<LinkType, Link>, OtsShape, Serializable, Identifiable
{

    /** */
    private static final long serialVersionUID = 20150101L;

    /**
     * The <b>timed</b> event type for pub/sub indicating the removal of a GTU from the link. <br>
     * Payload: Object[] {String gtuId, int count_after_removal}
     */
    public static final EventType GTU_REMOVE_EVENT = new EventType("LINK.GTU.REMOVE",
            new MetaData("GTU exited link", "GTU removed from link", new ObjectDescriptor[] {
                    new ObjectDescriptor("GTU id", "GTU id", String.class),
                    new ObjectDescriptor("Number of GTUs in link", "Resulting number of GTUs in link", Integer.class)}));

    /**
     * The <b>timed</b> event type for pub/sub indicating the addition of a GTU to the link. <br>
     * Payload: Object[] {String gtuId, int count_after_addition}
     */
    public static final EventType GTU_ADD_EVENT = new EventType("LINK.GTU.ADD",
            new MetaData("GTU entered link", "GTU added to link", new ObjectDescriptor[] {
                    new ObjectDescriptor("GTU id", "GTU id", String.class),
                    new ObjectDescriptor("Number of GTUs in link", "Resulting number of GTUs in link", Integer.class)}));

    /** the Network. */
    private final Network network;

    /** Link id. */
    private final String id;

    /** Start node (directional). */
    private final Node startNode;

    /** End node (directional). */
    private final Node endNode;

    /** Link type to indicate compatibility with GTU types. */
    private final LinkType linkType;

    /** Design line of the link. */
    private final OtsLine2d designLine;

    /** Elevation data. */
    private final ContinuousPiecewiseLinearFunction elevation;

    /** Absolute contour. */
    private final Polygon2d absoluteContour;

    /** Absolute contour. */
    private final Polygon2d relativeContour;

    /** The GTUs on this Link. */
    private final Set<Gtu> gtus = new LinkedHashSet<>();

    /** Location. */
    private DirectedPoint2d location;

    /**
     * Construct a new link.
     * @param network the network to which the link belongs
     * @param id the link id
     * @param startNode start node (directional)
     * @param endNode end node (directional)
     * @param linkType Link type to indicate compatibility with GTU types
     * @param designLine the OtsLine2d design line of the Link
     * @param elevation elevation given over fractional length, may be {@code null}.
     * @throws NetworkException if link already exists in the network, if name of the link is not unique, or if the start node
     *             or the end node of the link are not registered in the network.
     */
    public Link(final Network network, final String id, final Node startNode, final Node endNode, final LinkType linkType,
            final OtsLine2d designLine, final ContinuousPiecewiseLinearFunction elevation) throws NetworkException
    {
        Throw.whenNull(network, "network cannot be null");
        Throw.whenNull(id, "id cannot be null");
        Throw.whenNull(startNode, "startNode cannot be null (link %s)", id);
        Throw.whenNull(endNode, "endNode cannot be null (link %s)", id);
        Throw.whenNull(linkType, "linkType cannot be null (link %s)", id);
        Throw.whenNull(designLine, "designLine cannot be null (link %s)", id);

        this.network = network;
        this.id = id;
        this.startNode = startNode;
        this.endNode = endNode;
        this.linkType = linkType;
        this.startNode.addLink(this);
        this.endNode.addLink(this);
        this.designLine = designLine;
        this.elevation = elevation;
        this.absoluteContour = new Polygon2d(PolyLine2d.concatenate(this.designLine, this.designLine.reverse()).iterator());
        this.location = this.designLine.getLocationPointFractionExtended(0.5);
        this.relativeContour =
                new Polygon2d(OtsShape.toRelativeTransform(getLocation()).transform(this.absoluteContour.iterator()));
        this.network.addLink(this);
    }

    /**
     * Add a GTU to this link (e.g., for statistical purposes, or for a model on macro level). It is safe to add a GTU again. No
     * warning or error will be given. The GTU_ADD_EVENT will only be fired when the GTU was not already on the link.
     * @param gtu the GTU to add.
     */
    public final void addGTU(final Gtu gtu)
    {
        // TODO verify that gtu.getSimulator() equals getSimulator() ?
        if (!this.gtus.contains(gtu))
        {
            this.gtus.add(gtu);
            fireTimedEvent(Link.GTU_ADD_EVENT, new Object[] {gtu.getId(), this.gtus.size()},
                    gtu.getSimulator().getSimulatorTime());
        }
    }

    /**
     * Remove a GTU from this link. It is safe to try to remove a GTU again. No warning or error will be given. The
     * GTU_REMOVE_EVENT will only be fired when the GTU was on the link.
     * @param gtu the GTU to remove.
     */
    public final void removeGTU(final Gtu gtu)
    {
        // TODO verify that gtu.getSimulator() equals getSimulator() ?
        if (this.gtus.contains(gtu))
        {
            this.gtus.remove(gtu);
            fireTimedEvent(Link.GTU_REMOVE_EVENT, new Object[] {gtu.getId(), this.gtus.size()},
                    gtu.getSimulator().getSimulatorTime());
        }
    }

    /**
     * Provide a safe copy of the set of GTUs.
     * @return a safe copy of the set of GTUs
     */
    public final Set<Gtu> getGTUs()
    {
        return new LinkedHashSet<>(this.gtus);
    }

    /**
     * Provide the number of GTUs on this link.
     * @return the number of GTUs on this link
     */
    public final int getGTUCount()
    {
        return this.gtus.size();
    }

    /**
     * Returns whether the link is a connector. By default this returns {@code false}.
     * @return whether the link is a connector, by default this returns {@code false}.
     */
    public boolean isConnector()
    {
        return false;
    }

    /**
     * Return the network in which this link is registered. Cannot be null.
     * @return the network in which this link is registered
     */
    public Network getNetwork()
    {
        return this.network;
    }

    @Override
    public final String getId()
    {
        return this.id;
    }

    /**
     * Returns the start node.
     * @return start node.
     */
    public final Node getStartNode()
    {
        return this.startNode;
    }

    /**
     * Returns the end node.
     * @return end node.
     */
    public final Node getEndNode()
    {
        return this.endNode;
    }

    @Override
    public final LinkType getType()
    {
        return this.linkType;
    }

    /**
     * Returns the design line.
     * @return design line.
     */
    public final OtsLine2d getDesignLine()
    {
        return this.designLine;
    }

    @Override
    public Polygon2d getAbsoluteContour()
    {
        return this.absoluteContour;
    }

    @Override
    public Polygon2d getRelativeContour()
    {
        return this.relativeContour;
    }

    /**
     * Returns the simulator.
     * @return simulator.
     */
    public final OtsSimulatorInterface getSimulator()
    {
        return getNetwork().getSimulator();
    }

    /**
     * Returns the length of the link.
     * @return length of the link.
     */
    public final Length getLength()
    {
        return this.designLine.getTypedLength();
    }

    @Override
    public DirectedPoint2d getLocation()
    {
        return this.location;
    }

    @Override
    public Bounds2d getBounds()
    {
        return this.relativeContour.getBounds();
    }

    public Point2d getClosestPoint(Point2d point) {
        double minDistance = getDistance(point);
        for (int i = 0; i < this.absoluteContour.size() - 1; i++) {
            Point2d p1 = this.absoluteContour.get(i);
            Point2d p2 = this.absoluteContour.get(i + 1);

            if (distancePointToSegment(point, p1, p2) <= minDistance) {
                Point2d projectedPoint = projectPoint(point, p1, p2);
                return projectedPoint;
            }
        }
        return null;
    }

    public double lengthToClosestPoint(Point2d point) {
        double minDistance = getDistance(point);
        double cumulativeDistance = 0;
        for (int i = 0; i < this.absoluteContour.size() - 1; i++) {
            Point2d p1 = this.absoluteContour.get(i);
            Point2d p2 = this.absoluteContour.get(i + 1);
            double segmentLength = p1.distance(p1);

            if (distancePointToSegment(point, p1, p2) <= minDistance) {
                Point2d projectedPoint = projectPoint(point, p1, p2);
                double partiallySegmentLength = p1.distance(projectedPoint);
                cumulativeDistance += partiallySegmentLength;
                return cumulativeDistance;
            } else {
                cumulativeDistance += segmentLength;
            }
        }
        return -1;
    }

    public double getDistance(Point2d point) {
        // Gets the minimum distance of an absolute point to the absoluteContour.
        double minDistance = Double.MAX_VALUE;
        for (int i = 0; i < this.absoluteContour.size() - 1; i++) {
            Point2d p1 = this.absoluteContour.get(i);
            Point2d p2 = this.absoluteContour.get(i + 1);
            double distance = distancePointToSegment(point, p1, p2);
            minDistance = Math.min(minDistance, distance);
        }
        return minDistance;
    }

    private Point2d projectPoint(Point2d point, Point2d p1, Point2d p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        if (dx == 0 && dy == 0) {
            // a and b are the same point
            return p1;
        }

        // Project point p onto the line segment ab, computing parameterized t
        double t = ((point.x - p1.x) * dx + (point.y - p1.y) * dy) / (dx * dx + dy * dy);

        // Clamp t from 0 to 1
        t = Math.max(0, Math.min(1, t));

        // Compute projection point
        double projX = p1.x + t * dx;
        double projY = p1.y + t * dy;

        return new Point2d(projX, projY);
    }

    private double distancePointToSegment(Point2d p, Point2d a, Point2d b) {
        Point2d projectedPoint = projectPoint(p, a, b);
        // Distance from point to projection
        double distX = p.x - projectedPoint.x;
        double distY = p.y - projectedPoint.y;
        return Math.hypot(distX, distY);
    }

    /**
     * Returns the elevation at the given position.
     * @param position position.
     * @return elevation at the given position.
     */
    public Length getElevation(final Length position)
    {
        return getElevation(position.si / getLength().si);
    }

    /**
     * Returns the elevation at the given fractional position.
     * @param fractionalPosition fractional position.
     * @return elevation at the given fractional position.
     */
    public Length getElevation(final double fractionalPosition)
    {
        if (this.elevation == null)
        {
            return Length.ZERO;
        }
        return Length.instantiateSI(this.elevation.get(fractionalPosition));
    }

    /**
     * Returns the grade at the given position, given as delta_h / delta_x, as value without unit.
     * @param position position
     * @return grade at the given position
     */
    public double getGrade(final Length position)
    {
        return getGrade(position.si / getLength().si);
    }

    /**
     * Returns the grade at the given fractional position, given as delta_h / delta_x, as value without unit.
     * @param fractionalPosition fractional position
     * @return grade at the given fractional position
     */
    public double getGrade(final double fractionalPosition)
    {
        if (this.elevation == null)
        {
            return 0.0;
        }
        return this.elevation.getDerivative(fractionalPosition) / getLength().si;
    }

    @Override
    @SuppressWarnings("checkstyle:designforextension")
    public String toString()
    {
        return this.id.toString();
    }

    @Override
    @SuppressWarnings("checkstyle:designforextension")
    public int hashCode()
    {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((this.endNode == null) ? 0 : this.endNode.hashCode());
        result = prime * result + ((this.id == null) ? 0 : this.id.hashCode());
        result = prime * result + ((this.linkType == null) ? 0 : this.linkType.hashCode());
        result = prime * result + ((this.startNode == null) ? 0 : this.startNode.hashCode());
        return result;
    }

    @Override
    @SuppressWarnings({"checkstyle:designforextension", "checkstyle:needbraces"})
    public boolean equals(final Object obj)
    {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Link other = (Link) obj;
        if (this.endNode == null)
        {
            if (other.endNode != null)
                return false;
        }
        else if (!this.endNode.equals(other.endNode))
            return false;
        if (this.id == null)
        {
            if (other.id != null)
                return false;
        }
        else if (!this.id.equals(other.id))
            return false;
        if (this.linkType == null)
        {
            if (other.linkType != null)
                return false;
        }
        else if (!this.linkType.equals(other.linkType))
            return false;
        if (this.startNode == null)
        {
            if (other.startNode != null)
                return false;
        }
        else if (!this.startNode.equals(other.startNode))
            return false;
        return true;
    }

}
