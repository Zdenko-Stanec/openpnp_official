package org.openpnp.machine.photon;

import org.apache.commons.io.IOUtils;
import org.openpnp.ConfigurationListener;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.photon.exceptions.FeedFailureException;
import org.openpnp.machine.photon.exceptions.FeederHasNoLocationOffsetException;
import org.openpnp.machine.photon.exceptions.NoSlotAddressException;
import org.openpnp.machine.photon.exceptions.UnconfiguredSlotException;
import org.openpnp.machine.photon.protocol.ErrorTypes;
import org.openpnp.machine.photon.protocol.PhotonBus;
import org.openpnp.machine.photon.protocol.PhotonBusInterface;
import org.openpnp.machine.photon.protocol.commands.*;
import org.openpnp.machine.photon.sheets.FeederPropertySheet;
import org.openpnp.machine.photon.sheets.GlobalConfigPropertySheet;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.ReferenceFeeder;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.machine.reference.feeder.ReferenceStripFeeder;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Solutions;
import org.openpnp.spi.*;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.OpenCvUtils;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;

import javax.swing.*;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PhotonFeeder extends ReferenceFeeder {
    public static final String ACTUATOR_DATA_NAME = "PhotonFeederData";
    PhotonProperties photonProperties;

    @Attribute(required = false)
    protected String hardwareId;

    protected Integer slotAddress = null;

    @Attribute(required = false)
    protected int partPitch = 4;

    protected boolean initialized = false;

    @Element(required = false)
    private Location offset;

    private static PhotonBusInterface photonBus;

    @Attribute(required = false)
    protected boolean moveWhileFeeding = true;

    @Attribute(required = false)
    protected boolean feedAfterPick = false;

    /**
     * True when a tape advance has already been commanded (right after the previous pick,
     * see postPick()) and has not yet been consumed by a feed() call. Persisted so that a
     * normal OpenPnP restart does not cause a duplicate tape advance (which would waste
     * one component).
     */
    @Attribute(required = false)
    protected boolean feedPrepared = false;

    /**
     * expectedTimeToFeed (in milliseconds) reported by the feeder for the currently
     * prepared advance. Not persisted; a sensible bound is derived from partPitch if
     * OpenPnP was restarted in between.
     */
    private int preparedFeedExpectedMillis = 0;

    /**
     * Vision-Assisted Pick: after each feed, locate the tape sprocket holes with the
     * head camera and correct the pick location for the one pocket that is about to be
     * picked. Off by default; costs camera travel plus processing time per pick, so it
     * is intended for small parts (0201/01005) where pocket-level accuracy matters.
     */
    @Attribute(required = false)
    protected boolean visionAssistedPick = false;

    /**
     * Vision pipeline used by Vision-Assisted Pick to find the tape sprocket holes.
     * Seeded from the ReferenceStripFeeder default sprocket hole pipeline
     * (DetectCircularSymmetry); editable per feeder in the configuration wizard.
     */
    @Element(required = false)
    private CvPipeline pipeline = createDefaultPipeline();

    /**
     * XY pick correction (machine coordinates) determined by Vision-Assisted Pick for
     * the single pocket that is about to be picked. Deliberately transient and never
     * persisted: it is computed fresh after every feed, consumed by getPickLocation()
     * and discarded again at postPick(). Stateless per pick by design.
     */
    private transient Location visionOffset = null;

    /*
     * EIA-481 geometry of 8 mm carrier tape. These are properties of the tape standard,
     * not of the machine: sprocket holes are always diameter 1.5 mm on a 4 mm grid, the
     * pocket centerline is always 3.5 mm from the sprocket hole centerline, and pocket
     * centers are always offset 2 mm (dimension P2) from a sprocket hole center along
     * the tape, regardless of the part pitch (4 / 2 / 1 mm). All conversion between
     * pixels and millimeters happens at runtime through the camera's own units-per-pixel
     * calibration (camera General Configuration, Units Per Pixel incl. 3D / secondary)
     * - nothing in the vision code is expressed in pixels.
     */
    private static final double TAPE_SPROCKET_HOLE_DIAMETER_MM = 1.5;
    private static final double TAPE_SPROCKET_HOLE_PITCH_MM = 4.0;
    private static final double TAPE_POCKET_LATERAL_MM = 3.5;
    private static final double TAPE_POCKET_LONGITUDINAL_MM = 2.0;  // EIA-481 "P2"

    /** Sprocket hole search radius around the pick location. */
    private static final double VISION_SEARCH_RADIUS_MM = 12.0;
    /** Max plausible lateral correction; anything larger is treated as a vision failure. */
    private static final double VISION_MAX_LATERAL_CORRECTION_MM = 1.0;
    /** Max residual of a hole from the fitted tape line before it is discarded as an outlier. */
    private static final double VISION_MAX_LINE_RESIDUAL_MM = 0.3;
    /** Tolerance on the 4 mm sprocket grid when validating the detected holes. */
    private static final double VISION_GRID_TOLERANCE_MM = 0.4;

    public PhotonFeeder() {
        Configuration.get().addListener(new ConfigurationListener.Adapter() {
            @Override
            public void configurationLoaded(Configuration configuration) {
                photonProperties = new PhotonProperties(configuration.getMachine());

                // Ensure actuators are added to the machine when it has PhotonFeeders
                getDataActuator();
                populatePhotonBus();
            }
        });
    }

    public static void setBus(PhotonBusInterface bus) {
        photonBus = bus;
    }

    public static PhotonBusInterface getBus() {
        populatePhotonBus();
        return photonBus;
    }

    private static void populatePhotonBus() {
        if (photonBus != null) {
            return;
        }

        photonBus = new PhotonBus(0, getDataActuator());
    }

    @Override
    public Location getPickLocation() throws Exception {
        Location location = getNominalPickLocation();

        if (visionOffset != null) {
            // Vision-Assisted Pick correction for the one pocket about to be picked
            // (computed after the last feed, discarded again at postPick()).
            location = location.add(visionOffset);
        }

        return location;
    }

    /**
     * The configured (mechanically calibrated) pick location, without any
     * Vision-Assisted Pick correction applied.
     */
    private Location getNominalPickLocation() throws Exception {
        verifyFeederLocationIsFullyConfigured();

        return offset.offsetWithRotationFrom(getSlot().getLocation());
    }

    private void verifyFeederLocationIsFullyConfigured() throws NoSlotAddressException,
            UnconfiguredSlotException, FeederHasNoLocationOffsetException {
        if (slotAddress == null) {
            throw new NoSlotAddressException(
                    String.format("Photon Feeder with address %s has no address. Is it inserted?", hardwareId)
            );
        }

        if (getSlot().getLocation() == null) {
            throw new UnconfiguredSlotException(
                    String.format("The slot at address %s has no location configured.", slotAddress)
            );
        }

        if (offset == null) {
            throw new FeederHasNoLocationOffsetException(
                    String.format("Photon Feeder with address %s has no location offset.", hardwareId)
            );
        }
    }

    public void setOffset(Location offsets) {
        Object oldValue = this.offset;
        this.offset = offsets;
        firePropertyChange("offsets", oldValue, offsets);
    }

    public Location getOffset() {
        return offset;
    }

    public boolean getMoveWhileFeeding() {
        return moveWhileFeeding;
    }

    public void setMoveWhileFeeding(boolean moveWhileFeeding) {
        boolean oldValue = this.moveWhileFeeding;
        this.moveWhileFeeding = moveWhileFeeding;
        firePropertyChange("moveWhileFeeding", oldValue, moveWhileFeeding);
    }

    public boolean getFeedAfterPick() {
        return feedAfterPick;
    }

    public void setFeedAfterPick(boolean feedAfterPick) {
        boolean oldValue = this.feedAfterPick;
        this.feedAfterPick = feedAfterPick;
        firePropertyChange("feedAfterPick", oldValue, feedAfterPick);
    }

    public boolean isFeedPrepared() {
        return feedPrepared;
    }

    public void setFeedPrepared(boolean feedPrepared) {
        boolean oldValue = this.feedPrepared;
        this.feedPrepared = feedPrepared;
        firePropertyChange("feedPrepared", oldValue, feedPrepared);
    }

    public boolean isVisionAssistedPick() {
        return visionAssistedPick;
    }

    public void setVisionAssistedPick(boolean visionAssistedPick) {
        boolean oldValue = this.visionAssistedPick;
        this.visionAssistedPick = visionAssistedPick;
        firePropertyChange("visionAssistedPick", oldValue, visionAssistedPick);
    }

    public CvPipeline getPipeline() {
        return pipeline;
    }

    public void resetPipeline() {
        pipeline = createDefaultPipeline();
    }

    /**
     * Returns the vision pipeline with the sprocket hole geometry properties applied.
     * The Lengths are converted to pixels inside the DetectCircularSymmetry stage
     * through the camera, i.e. through the machine's own units-per-pixel calibration
     * (camera General Configuration, Units Per Pixel) - never through fixed pixel values.
     */
    public CvPipeline getPreparedPipeline(Camera camera) {
        pipeline.setProperty("camera", camera);
        pipeline.setProperty("feeder", this);
        pipeline.setProperty("sprocketHole.diameter",
                new Length(TAPE_SPROCKET_HOLE_DIAMETER_MM, LengthUnit.Millimeters));
        pipeline.setProperty("sprocketHole.maxDistance",
                new Length(VISION_SEARCH_RADIUS_MM, LengthUnit.Millimeters));
        return pipeline;
    }

    private static CvPipeline createDefaultPipeline() {
        try {
            String xml = IOUtils.toString(ReferenceStripFeeder.class
                    .getResource("ReferenceStripFeeder-DefaultPipeline.xml"));
            return new CvPipeline(xml);
        }
        catch (Exception e) {
            throw new Error(e);
        }
    }

    @Override
    public void findIssues(Solutions solutions) {
        super.findIssues(solutions);

        if (hardwareId == null) {
            return;
        }

        if (slotAddress != null && getSlot().getLocation() == null) {
            solutions.add(new Solutions.PlainIssue(
                    this,
                    "Feeder slot has no configured location",
                    "Select the feeder in the Feeders tab and make sure the slot has a set location",
                    Solutions.Severity.Error,
                    "https://github.com/openpnp/openpnp/wiki/Photon-Feeder#slots-and-feeder-locations"
            ));
        }

        if (offset == null) {
            solutions.add(new Solutions.PlainIssue(
                    this,
                    "Feeder has no configured offset",
                    "Select the feeder in the Feeders tab and make sure the feeder has an offset location from the slot",
                    Solutions.Severity.Error,
                    "https://github.com/openpnp/openpnp/wiki/Photon-Feeder#slots-and-feeder-locations"
            ));
        }
    }

    @Override
    public void prepareForJob(boolean visit) throws Exception {
        for (int i = 0; i <= photonProperties.getFeederCommunicationMaxRetry(); i++) {
            findSlotAddressIfNeeded();

            initializeIfNeeded();

            if (initialized) {
                verifyFeederLocationIsFullyConfigured();

                super.prepareForJob(visit);
                return;
            }
        }

        throw new Exception("Failed to find and initialize the feeder");
    }

    public void findSlotAddress() throws Exception {
        findSlotAddress(true);
    }

    private void findSlotAddress(boolean force) throws Exception {
        if (slotAddress != null && !force) {
            return;
        }

        GetFeederAddress getFeederAddress = new GetFeederAddress(hardwareId);
        GetFeederAddress.Response response = getFeederAddress.send(photonBus);

        if (response == null) {
            setSlotAddress(null);
            return;
        }
        setSlotAddress(response.fromAddress);
    }

    private void findSlotAddressIfNeeded() throws Exception {
        findSlotAddress(false);
    }

    public void initializeIfNeeded() throws Exception {
        if (initialized || slotAddress == null) {
            return;
        }

        InitializeFeeder initializeFeeder = new InitializeFeeder(slotAddress, hardwareId);
        InitializeFeeder.Response response = initializeFeeder.send(photonBus);

        if (response == null) {
            slotAddress = null;
        } else if (response.error == ErrorTypes.WRONG_FEEDER_UUID) {
            PhotonFeeder otherFeeder = findByHardwareId(response.uuid);

            // If we don't know about that feeder, let's go ahead and create it
            if (otherFeeder == null) {
                otherFeeder = new PhotonFeeder();
                otherFeeder.setHardwareId(response.uuid);
                Configuration.get().getMachine().addFeeder(otherFeeder);
            }

            // This other feeder is in the slot we thought we were
            otherFeeder.setSlotAddress(response.fromAddress);
        } else {
            initialized = true;
        }
    }

    static Actuator getDataActuator() {
        Machine machine = Configuration.get().getMachine();

        Actuator actuator = machine.getActuatorByName(ACTUATOR_DATA_NAME);

        if (actuator == null) {
            actuator = createDefaultActuator(machine);
        }

        return actuator;
    }

    private static Actuator createDefaultActuator(Machine machine) {
        Actuator actuator;
        actuator = new ReferenceActuator();
        actuator.setName(ACTUATOR_DATA_NAME);

        for (Driver driver : machine.getDrivers()) {
            if(! (driver instanceof GcodeDriver)) {
                continue;
            }
            GcodeDriver gcodeDriver = (GcodeDriver) driver;
            gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_COMMAND, "M485 {Value}");
            gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_REGEX, "rs485-reply: (?<Value>.*)");
            break;  // Only set this on 1 GCodeDriver
        }

        try {
            machine.addActuator(actuator);
        } catch (Exception exception) {
            exception.printStackTrace(); // TODO Probably need to log this, figure out why it can happen first
        }
        return actuator;
    }

    /**
     * Sends the MoveFeedForward command to the feeder, including slot discovery and
     * initialization retries. The feeder acknowledges the command immediately and then
     * performs the tape movement on its own; use waitForFeedCompletion() to wait for the
     * mechanical move to actually finish.
     *
     * @return the expectedTimeToFeed reported by the feeder, in milliseconds.
     */
    private int sendFeedCommand(int distance_mm) throws Exception {
        // SAFETY: never start the tape while the machine is still moving. The
        // PhotonFeederData actuator may be configured without machine coordination
        // (so that status polling can run during head motion), in which case the feed
        // command could otherwise be sent while a nozzle is still down in - or
        // retracting out of - the pocket area with a picked part, and the advancing
        // tape could clip it. At stillstand every nozzle is at safe Z, which clears
        // everything by definition. This gate covers BOTH the normal feed path and
        // Feed After Pick, since both send their feed command through this method.
        Configuration.get().getMachine().getMotionPlanner()
                .waitForCompletion(null, MotionPlanner.CompletionType.WaitForStillstand);

        for (int i = 0; i <= photonProperties.getFeederCommunicationMaxRetry(); i++) {
            findSlotAddressIfNeeded();
            initializeIfNeeded();

            if (!initialized) {
                continue;
            }

            verifyFeederLocationIsFullyConfigured();

            MoveFeedForward moveFeedForward = new MoveFeedForward(slotAddress, distance_mm * 10);
            MoveFeedForward.Response moveFeedForwardResponse = moveFeedForward.send(photonBus);

            if (moveFeedForwardResponse == null) {
                slotAddress = null;
                initialized = false;
                throw new FeedFailureException("Feed command timed out");
            } else if (moveFeedForwardResponse.error == ErrorTypes.UNINITIALIZED_FEEDER) {
                slotAddress = null;
                initialized = false;
                continue;  // We'll initialize it on a retry
            }

            return moveFeedForwardResponse.expectedTimeToFeed;
        }

        throw new FeedFailureException("Failed to feed for an unknown reason. Is the feeder inserted?");
    }

    /**
     * Polls the feeder until it reports that the last commanded feed completed. While
     * waiting, optionally (moveWhileFeeding) moves the nozzle at safe Z over the pick
     * location, so that machine travel and tape movement happen in parallel.
     */
    private void waitForFeedCompletion(Nozzle nozzle, int expectedTimeToFeedMillis) throws Exception {
        // Start moving the nozzle toward the pick location IMMEDIATELY, before the
        // first polling sleep, so machine travel and tape movement start together
        // (previously the move was only issued after the first 50ms sleep).
        if (nozzle != null && Configuration.get().getMachine().isHomed() && getMoveWhileFeeding()) {
            HeadMountable premove = nozzle;
            Location premoveLocation =
                    getPickLocation().deriveLengths(null, null, nozzle.getEffectiveSafeZ(), null);
            if (visionAssistedPick) {
                // With Vision-Assisted Pick it is the CAMERA that must arrive over the
                // pick area first (the vision snapshot precedes the pick), so pre-move
                // the camera instead of the nozzle. Camera travel then hides inside the
                // tape feed time and the snapshot can be taken the moment the feed
                // confirms. The location keeps the tape Z: for a camera with a virtual
                // Z axis this sets the viewing plane for correct units per pixel.
                premove = nozzle.getHead().getDefaultCamera();
                premoveLocation = getPickLocation().derive(null, null, null, 0.0);
            }
            MovableUtils.moveToLocationAtSafeZ(premove, premoveLocation);
            // Commit the planned motion to the driver WITHOUT waiting for it: with
            // continuous motion enabled, the motion planner would otherwise hold the
            // move in its look-ahead queue while this thread blocks polling the feeder,
            // and the head would only start moving after the feed completed.
            // CommandStillstand sends the plan for immediate execution (ending settled
            // at the target location) and returns right away, so head travel and tape
            // movement genuinely run in parallel.
            Configuration.get().getMachine().getMotionPlanner()
                    .waitForCompletion(premove, MotionPlanner.CompletionType.CommandStillstand);
        }

        // The feeder gives us expectedTimeToFeed, but it is way too conservative.
        // Use expectedTimeToFeed to bound how long we will wait,
        // but use polling to check the status of the feed.
        Duration expectedFeedDuration = Duration.ofMillis(expectedTimeToFeedMillis);
        long endTimeNanos = System.nanoTime() + expectedFeedDuration.toNanos() * 3;
        for (int j = 0; j <= photonProperties.getFeederCommunicationMaxRetry() || System.nanoTime() <= endTimeNanos; j++) {
            if (j > 0) {
                Thread.sleep(50); // MAGIC: this feels like a good number, there is no particular reason it is this way.
            }
            // The first status request is sent immediately: a bridge/feeder that answers
            // status event-driven (on feed completion) ends the wait as early as possible,
            // and an already-completed prepared advance is confirmed in a few milliseconds.

            MoveFeedStatus moveFeedStatus = new MoveFeedStatus(slotAddress);
            MoveFeedStatus.Response moveFeedStatusResponse = moveFeedStatus.send(photonBus);

            if (moveFeedStatusResponse == null) {
                continue; // Timeout. The feeder may be unable to answer while it is moving. Retry after delay.
            }

            if (moveFeedStatusResponse.error == ErrorTypes.NONE) {
                return;
            } else if (moveFeedStatusResponse.error == ErrorTypes.COULD_NOT_REACH) {
                throw new FeedFailureException("Feeder could not reach its destination.");
            }
            // ErrorTypes.FEEDING_IN_PROGRESS and any unknown status: keep polling until
            // the time bound runs out.
        }

        throw new FeedFailureException("Feeder timed out when we requested a feed status update.");
    }

    /**
     * Timeout bound (milliseconds) used for a prepared advance when the
     * expectedTimeToFeed reported at postPick() time is no longer known (e.g. after an
     * OpenPnP restart).
     */
    private int preparedFeedFallbackMillis() {
        return Math.max(700, partPitch * 100);
    }

    private void feed(Nozzle nozzle, int distance_mm) throws Exception {
        if (feedPrepared) {
            // A tape advance was already commanded right after the previous pick
            // (Feed After Pick, see postPick()). Make sure it has completed.
            setFeedPrepared(false);
            int expected = preparedFeedExpectedMillis > 0 ? preparedFeedExpectedMillis
                    : preparedFeedFallbackMillis();
            preparedFeedExpectedMillis = 0;
            waitForFeedCompletion(nozzle, expected);

            if (distance_mm == partPitch) {
                // This feed cycle was already performed by the prepared advance.
                Logger.trace("Feeder {} feed satisfied by prepared advance (Feed After Pick).", getName());
                return;
            }
            // A different, explicit distance was requested (e.g. Feed 1mm). The prepared
            // advance is a physical fact; perform the requested feed in addition to it.
        }

        int expectedTimeToFeed = sendFeedCommand(distance_mm);
        waitForFeedCompletion(nozzle, expectedTimeToFeed);
    }

    @Override
    public void feed(Nozzle nozzle) throws Exception {
        // Vision-Assisted Pick: any correction left over from a previous cycle is
        // invalid the moment a new feed cycle starts. It is recomputed below.
        visionOffset = null;

        switch (getFeedOptions()) {
        case Normal:
            feed(nozzle, partPitch);
            break;
        case SkipNext:
            setFeedOptions(FeedOptions.Normal);
            break;
        case Disable:
            break;
        }

        // Vision-Assisted Pick runs strictly after the tape is guaranteed stationary
        // again: the feed above is confirmed mechanically complete (with Feed After
        // Pick, the prepared advance was verified complete inside feed(nozzle, pitch)).
        // Only in a job context (nozzle != null): the manual Feed button and
        // feedOneMm() must never cause any machine motion (operator safety).
        if (visionAssistedPick && nozzle != null) {
            obtainVisionPickCorrection(nozzle);
        }
    }

    public void feedOneMm() throws Exception {
        feed(null, 1);
    }

    /**
     * Feed After Pick: right after a pick, command the tape advance for the next part and
     * return without waiting for it. The advance then runs while the machine leaves for
     * alignment and placement. The next feed() call for this feeder only has to verify
     * that the advance completed, which usually takes no time at all.
     */
    @Override
    public void postPick(Nozzle nozzle) throws Exception {
        // Vision-Assisted Pick: the correction was only ever valid for the pocket that
        // has just been picked. Discard it - stateless per pick, nothing is stored.
        visionOffset = null;

        if (!feedAfterPick) {
            return;
        }

        if (getFeedOptions() != FeedOptions.Normal) {
            // Feeding is disabled or the next feed is to be skipped: do not advance the tape.
            return;
        }

        try {
            // Note: sendFeedCommand() waits for machine stillstand before the tape is
            // started, i.e. the Z retract with the freshly picked part is guaranteed to
            // have physically completed (nozzle at safe Z) before any tape movement.

            if (feedPrepared) {
                // The pocket that was prepared earlier has just been consumed by this pick
                // (this can happen with pick retries, or with multiple picks from the same
                // feeder in one planning cycle). Make sure that advance is fully done
                // before commanding a new one.
                setFeedPrepared(false);
                int expected = preparedFeedExpectedMillis > 0 ? preparedFeedExpectedMillis
                        : preparedFeedFallbackMillis();
                waitForFeedCompletion(null, expected);
            }

            preparedFeedExpectedMillis = sendFeedCommand(partPitch);
            setFeedPrepared(true);
            Logger.trace("Feeder {} started Feed After Pick tape advance.", getName());
        } catch (Exception e) {
            // A failed pre-feed must never fail the job; the part is already on the
            // nozzle. The next feed() call will simply perform a normal, blocking feed.
            preparedFeedExpectedMillis = 0;
            Logger.warn(e, "Feed After Pick failed for feeder {}. The next feed will be performed normally.", getName());
        }
    }

    /**
     * Vision-Assisted Pick: with the tape guaranteed stationary, take an image of the
     * pick area with the head camera, locate the tape sprocket holes and derive a
     * corrected pick location for the pocket about to be picked. The correction is
     * stored in the transient visionOffset (consumed by getPickLocation()) and
     * discarded again at postPick(): nothing is ever persisted, every pick gets a
     * fresh, independent correction.
     *
     * This method never throws: on any vision problem it logs a warning and leaves the
     * configured pick location in effect. The mechanically calibrated location normally
     * still picks fine, and vacuum sensing, bottom vision and the feed/pick retry
     * mechanisms catch genuinely failed picks.
     */
    private void obtainVisionPickCorrection(Nozzle nozzle) {
        Camera camera;
        try {
            camera = nozzle.getHead().getDefaultCamera();
        }
        catch (Exception e) {
            visionOffset = null;
            Logger.warn(e, "Feeder {}: Vision-Assisted Pick failed; picking at the "
                    + "configured location.", getName());
            return;
        }
        obtainVisionPickCorrection(camera);
    }

    private void obtainVisionPickCorrection(Camera camera) {
        visionOffset = null;
        Location cameraLocation = null;
        Location unitsPerPixel = null;
        BufferedImage preview = null;
        Location nominal = null;
        try {
            nominal = getNominalPickLocation().convertToUnits(LengthUnit.Millimeters);

            // Move the camera over the nominal pick location, INCLUDING the tape Z
            // (pick level). For a camera with a (virtual) Z axis this sets the viewing
            // plane, so on-screen reticles and any in-pipeline pixel conversion use the
            // units per pixel at tape height rather than at the default working plane.
            // If Move While Feeding already pre-moved the camera here, this is a no-op.
            MovableUtils.moveToLocationAtSafeZ(camera, nominal.derive(null, null, null, 0.0));
            // Wait for the WHOLE machine to be physically at rest, position confirmed
            // by the controller. Important: waiting scoped to the camera can return
            // immediately when its motion was already committed with CommandStillstand
            // by the Move While Feeding pre-move (observed on the machine as captures
            // taken in flight over the PCB), so use the same global gate that is
            // proven in sendFeedCommand().
            Configuration.get().getMachine().getMotionPlanner()
                    .waitForCompletion(null, MotionPlanner.CompletionType.WaitForStillstand);

            // Defense in depth: verify the camera has actually arrived over the pick
            // location before taking the image. With Location Confirmation, the
            // location below is the controller-reported one.
            Location actual = camera.getLocation().convertToUnits(LengthUnit.Millimeters);
            double arrivalError = Math.hypot(actual.getX() - nominal.getX(),
                    actual.getY() - nominal.getY());
            if (arrivalError > 0.5) {
                throw new Exception(String.format(Locale.US,
                        "Camera did not arrive at the pick location (%.3f mm away).",
                        arrivalError));
            }

            // Units per pixel at the tape plane, taken from the machine's camera
            // calibration (camera General Configuration, Units Per Pixel). With the 3D
            // (dual height) calibration configured, this interpolates to the tape
            // height. Z is passed explicitly, so the result is correct even for a
            // camera without a virtual Z axis.
            if (!camera.isUnitsPerPixelAtZCalibrated()) {
                Logger.warn("Feeder {}: camera {} has no 3D (dual height) units per pixel "
                                + "calibration; using the primary units per pixel. The vision "
                                + "correction will carry a scale error at tape height - please "
                                + "calibrate the secondary units per pixel and enable 3D.",
                        getName(), camera.getName());
            }
            unitsPerPixel = camera.getUnitsPerPixel(nominal.getLengthZ())
                    .convertToUnits(LengthUnit.Millimeters);

            // Run the sprocket hole pipeline and convert the detected circles to
            // machine coordinates (millimeters), using the units per pixel valid at
            // TAPE height and the standard pixel-to-machine sign convention (see
            // VisionUtils.getPixelCenterOffsets()).
            List<Location> holes;
            try (CvPipeline pipeline = getPreparedPipeline(camera)) {
                pipeline.process();

                // Keep a copy of the pipeline image (with the detected circles drawn,
                // if the DrawCircles stage is enabled) for the operator preview.
                preview = OpenCvUtils.toBufferedImage(pipeline.getWorkingImage());

                List<CvStage.Result.Circle> circles = pipeline
                        .getExpectedResult(VisionUtils.PIPELINE_RESULTS_NAME)
                        .getExpectedListModel(CvStage.Result.Circle.class,
                                new Exception("No tape sprocket holes found."));

                cameraLocation = camera.getLocation().convertToUnits(LengthUnit.Millimeters);
                holes = new ArrayList<>();
                for (CvStage.Result.Circle circle : circles) {
                    double dx = (circle.x - camera.getWidth() / 2.0) * unitsPerPixel.getX();
                    double dy = -(circle.y - camera.getHeight() / 2.0) * unitsPerPixel.getY();
                    holes.add(cameraLocation
                            .add(new Location(LengthUnit.Millimeters, dx, dy, 0, 0)));
                }
            }

            computeVisionOffset(nominal, holes);

            // Operator preview: nominal pick location in red, the vision-corrected
            // pick location (pocket derived from the holes and the part pitch) in
            // green, drawn over the pipeline image with the detected holes.
            drawPickLocationMarker(preview, camera, cameraLocation, unitsPerPixel,
                    nominal, Color.RED, false);
            drawPickLocationMarker(preview, camera, cameraLocation, unitsPerPixel,
                    nominal.add(visionOffset), Color.GREEN, true);
            drawOverlayText(preview, String.format(Locale.US, "VAP dX %+.3f  dY %+.3f mm",
                    visionOffset.getX(), visionOffset.getY()));
        }
        catch (Exception e) {
            visionOffset = null;
            Logger.warn(e, "Feeder {}: Vision-Assisted Pick failed; picking at the "
                    + "configured location.", getName());
            // On failure, still mark the (unchanged) nominal pick location in red.
            drawPickLocationMarker(preview, camera, cameraLocation, unitsPerPixel,
                    nominal, Color.RED, true);
        }
        showVisionPreview(camera, preview);
    }

    /**
     * Draws a crosshair (and optionally a 1 mm circle) at the given machine location
     * onto the vision preview image. Skipped silently if any input is unavailable.
     */
    private void drawPickLocationMarker(BufferedImage image, Camera camera,
            Location cameraLocation, Location unitsPerPixel, Location location,
            Color color, boolean withCircle) {
        if (image == null || camera == null || cameraLocation == null
                || unitsPerPixel == null || location == null) {
            return;
        }
        int x = (int) Math.round(camera.getWidth() / 2.0
                + (location.getX() - cameraLocation.getX()) / unitsPerPixel.getX());
        int y = (int) Math.round(camera.getHeight() / 2.0
                - (location.getY() - cameraLocation.getY()) / unitsPerPixel.getY());
        int arm = (int) Math.round(0.5 / unitsPerPixel.getX());     // 0.5 mm crosshair arms
        int radius = (int) Math.round(0.5 / unitsPerPixel.getX());  // 1 mm diameter circle
        Graphics2D g = image.createGraphics();
        try {
            g.setStroke(new BasicStroke(2f));
            g.setColor(color);
            g.drawLine(x - arm, y, x + arm, y);
            g.drawLine(x, y - arm, x, y + arm);
            if (withCircle) {
                g.drawOval(x - radius, y - radius, radius * 2, radius * 2);
            }
        }
        finally {
            g.dispose();
        }
    }

    /**
     * Draws a text line (green with black shadow for readability) into the top left
     * corner of the vision preview image. Skipped silently if the image is unavailable.
     */
    private void drawOverlayText(BufferedImage image, String text) {
        if (image == null) {
            return;
        }
        Graphics2D g = image.createGraphics();
        try {
            g.setFont(new Font(Font.DIALOG, Font.BOLD, 20));
            g.setColor(Color.BLACK);
            g.drawString(text, 11, 26);
            g.setColor(Color.GREEN);
            g.drawString(text, 10, 25);
        }
        finally {
            g.dispose();
        }
    }

    /**
     * Shows the vision preview image in the camera view for a moment, if the UI is up.
     */
    private void showVisionPreview(Camera camera, BufferedImage image) {
        if (camera == null || image == null || MainFrame.get() == null) {
            return;
        }
        try {
            MainFrame.get().getCameraViews().getCameraView(camera)
                    .showFilteredImage(image, 2000);
        }
        catch (Exception e) {
            // Not running in the UI; that is fine.
        }
    }

    /**
     * Derives the pick correction from the detected sprocket hole locations
     * (millimeters, machine coordinates) for the pocket at the nominal pick location:
     *
     * 1. A least-squares line is fitted through the hole centers; its direction is the
     *    tape axis. Outliers are discarded by residual (a dark part in a pocket
     *    mis-detected as a hole would sit about 3.5 mm off the hole line, a partially
     *    occluded hole at the feeder window edge lands off-grid).
     * 2. The hole spacing is validated against the 4 mm sprocket grid.
     * 3. Lateral: the pocket centerline is always exactly 3.5 mm from the hole
     *    centerline (EIA-481), so the corrected pick point is placed at exactly that
     *    distance from the fitted line, on the same side as the nominal pick point.
     *    This absorbs lateral tape wander and local tape rotation in one step.
     * 4. Longitudinal: pocket centers sit at (hole + 2 mm + m * partPitch) along the
     *    tape (EIA-481 P2 = 2 mm). The candidate nearest the nominal pick point is the
     *    pocket about to be picked; the remaining distance is the correction. This is
     *    inherently bounded to +/- partPitch / 2 and the candidate set is symmetric
     *    under tape direction reversal, so the feed direction never needs to be known.
     *
     * Any implausible result throws, which the caller turns into "no correction".
     */
    private void computeVisionOffset(Location nominal, List<Location> holes) throws Exception {
        List<double[]> points = new ArrayList<>();
        for (Location hole : holes) {
            points.add(new double[] {hole.getX(), hole.getY()});
        }

        double ux, uy, cx, cy;
        while (true) {
            int n = points.size();
            if (n < 2) {
                throw new Exception(String.format(
                        "Only %d usable sprocket hole(s) detected, need at least 2.", n));
            }
            cx = 0;
            cy = 0;
            for (double[] p : points) {
                cx += p[0];
                cy += p[1];
            }
            cx /= n;
            cy /= n;
            double sxx = 0, sxy = 0, syy = 0;
            for (double[] p : points) {
                double dx = p[0] - cx, dy = p[1] - cy;
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
            }
            // Principal axis of the hole centers = tape axis.
            double angle = 0.5 * Math.atan2(2.0 * sxy, sxx - syy);
            ux = Math.cos(angle);
            uy = Math.sin(angle);

            // Perpendicular residuals; discard the worst outlier and refit while
            // enough holes remain.
            int worst = -1;
            double worstResidual = 0;
            for (int i = 0; i < n; i++) {
                double[] p = points.get(i);
                double r = Math.abs(-(p[0] - cx) * uy + (p[1] - cy) * ux);
                if (r > worstResidual) {
                    worstResidual = r;
                    worst = i;
                }
            }
            if (worstResidual > VISION_MAX_LINE_RESIDUAL_MM) {
                if (n > 2) {
                    Logger.trace("Feeder {}: discarding sprocket hole outlier, residual {} mm.",
                            getName(), String.format("%.3f", worstResidual));
                    points.remove(worst);
                    continue;
                }
                throw new Exception(String.format(
                        "Sprocket holes do not form a line (residual %.3f mm).", worstResidual));
            }
            break;
        }
        double nx = -uy, ny = ux;  // lateral (perpendicular) unit vector

        // Validate the 4 mm sprocket grid: neighboring projected holes must be close to
        // a whole multiple of the hole pitch and at least one pitch apart (rejects
        // duplicate and false circle detections).
        List<Double> s = new ArrayList<>();
        for (double[] p : points) {
            s.add((p[0] - cx) * ux + (p[1] - cy) * uy);
        }
        s.sort(null);
        for (int i = 1; i < s.size(); i++) {
            double d = s.get(i) - s.get(i - 1);
            double gridError = Math.abs(d - TAPE_SPROCKET_HOLE_PITCH_MM
                    * Math.round(d / TAPE_SPROCKET_HOLE_PITCH_MM));
            if (gridError > VISION_GRID_TOLERANCE_MM
                    || d < TAPE_SPROCKET_HOLE_PITCH_MM - VISION_GRID_TOLERANCE_MM) {
                throw new Exception(String.format(
                        "Detected holes do not match the 4 mm sprocket grid (spacing %.3f mm).",
                        d));
            }
        }

        // Lateral correction: the corrected pick point lies exactly 3.5 mm from the
        // fitted hole line, on the same side of it as the nominal pick point.
        double ex = nominal.getX() - cx, ey = nominal.getY() - cy;
        double lateralDistance = ex * nx + ey * ny;  // signed distance nominal <-> hole line
        double side = lateralDistance >= 0 ? 1.0 : -1.0;
        double lateralCorrection = side * TAPE_POCKET_LATERAL_MM - lateralDistance;
        if (Math.abs(lateralCorrection) > VISION_MAX_LATERAL_CORRECTION_MM) {
            throw new Exception(String.format(
                    "Implausible lateral correction %.3f mm (hole line found %.3f mm from the "
                            + "pick location, expected 3.5 mm).",
                    lateralCorrection, Math.abs(lateralDistance)));
        }

        // Longitudinal correction: nearest pocket candidate (hole + P2 + m * partPitch).
        double sNominal = ex * ux + ey * uy;
        double longitudinalCorrection = Double.NaN;
        for (double si : s) {
            for (int m = -16; m <= 16; m++) {
                double candidate = si + TAPE_POCKET_LONGITUDINAL_MM + m * partPitch;
                double delta = candidate - sNominal;
                if (Double.isNaN(longitudinalCorrection)
                        || Math.abs(delta) < Math.abs(longitudinalCorrection)) {
                    longitudinalCorrection = delta;
                }
            }
        }
        if (Double.isNaN(longitudinalCorrection)
                || Math.abs(longitudinalCorrection) > partPitch / 2.0 + 0.001) {
            throw new Exception(String.format(
                    "Implausible longitudinal correction %.3f mm.", longitudinalCorrection));
        }

        double offsetX = longitudinalCorrection * ux + lateralCorrection * nx;
        double offsetY = longitudinalCorrection * uy + lateralCorrection * ny;
        visionOffset = new Location(LengthUnit.Millimeters, offsetX, offsetY, 0, 0);
        Logger.trace("Feeder {}: Vision-Assisted Pick correction lateral {} mm, longitudinal {} "
                        + "mm, machine offset ({}, {}) mm, tape axis {} deg, {} hole(s) used.",
                getName(),
                String.format("%.4f", lateralCorrection),
                String.format("%.4f", longitudinalCorrection),
                String.format("%.4f", offsetX),
                String.format("%.4f", offsetY),
                String.format("%.2f", Math.toDegrees(Math.atan2(uy, ux))),
                points.size());
    }

    @Override
    public String getPropertySheetHolderTitle() {
        String classSimpleName = getClass().getSimpleName();
        if (hardwareId == null) {
            return String.format("Unconfigured %s", classSimpleName);
        } else {
            return String.format("%s %s", classSimpleName, getName());
        }
    }

    @Override
    public PropertySheet[] getPropertySheets() {
        List<PropertySheet> sheets = new ArrayList<>();

        if (hardwareId != null) {
            sheets.add(new FeederPropertySheet(this));
        }

        sheets.add(new GlobalConfigPropertySheet());

        return sheets.toArray(new PropertySheet[0]);
    }

    @Override
    public PropertySheetHolder[] getChildPropertySheetHolders() {
        return new PropertySheetHolder[0];
    }

    @Override
    public Action[] getPropertySheetHolderActions() {
        return new Action[0];
    }

    @Override
    public Wizard getConfigurationWizard() {
        return null;
    }

    @Override
    public String getName() {
        if (hardwareId == null) {
            return String.format("Unconfigured %s", getClass().getSimpleName());
        }

        StringBuilder result = new StringBuilder();
        result.append(name);
        result.append(" (Slot: ");

        if (slotAddress == null) {
            result.append("None");
        } else {
            result.append(slotAddress);
        }

        result.append(")");

        return result.toString();
    }

    @Override
    public void setName(String name) {
        Matcher matcher = Pattern.compile("(\\(Slot: [\\w+]+\\))").matcher(name);
        while (matcher.find()) {
            name = name.replace(matcher.group(), "");
        }

        name = name.trim();

        super.setName(name);
    }

    /**
     * The PhotonFeeder assumes you have a physical slot that is numbered 1 - 254. That
     * value is also used in the protocol as the address of the feeder once the feeder
     * is initialized.
     *
     * @return The slot address of this feeder or null if it doesn't have one.
     */
    public Integer getSlotAddress() {
        return slotAddress;
    }

    public PhotonFeederSlots.Slot getSlot() {
        if (slotAddress == null) {
            return null;
        }

        return photonProperties.getFeederSlots().getSlot(slotAddress);
    }

    public void setSlotAddress(Integer slotAddress) {
        PhotonFeederSlots.Slot oldSlot = getSlot();
        Integer oldValue = this.slotAddress;
        String oldName = this.getName();

        if (slotAddress != null) {
            // Find any other photon feeders and if they have this slot address, set their address to null
            PhotonFeeder otherFeeder = findBySlotAddress(slotAddress);
            if (otherFeeder != null) {
                otherFeeder.slotAddress = null;
                otherFeeder.initialized = false;
            }
        }

        this.slotAddress = slotAddress;

        firePropertyChange("slotAddress", oldValue, slotAddress);
        firePropertyChange("slot", oldSlot, getSlot());
        firePropertyChange("name", oldName, getName());
    }

    public String getHardwareId() {
        return hardwareId;
    }

    public void setHardwareId(String hardwareId) {
        String oldValue = this.hardwareId;
        this.hardwareId = hardwareId;

        if (getClass().getSimpleName().equals(name)) {
            name = hardwareId;
        }

        firePropertyChange("hardwareId", oldValue, hardwareId);
    }

    @Override
    public boolean isEnabled() {
        return super.isEnabled() &&
                hardwareId != null &&
                partId != null &&
                slotAddress != null &&
                offset != null &&
                getSlot().getLocation() != null;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void setPartPitch(int partPitch) {
        this.partPitch = partPitch;
    }

    public int getPartPitch() {
        return partPitch;
    }

    public static PhotonFeeder findByHardwareId(String hardwareId) {
        for (Feeder feeder : Configuration.get().getMachine().getFeeders()) {
            if (!(feeder instanceof PhotonFeeder)) {
                continue;
            }

            PhotonFeeder photonFeeder = (PhotonFeeder) feeder;

            // Are we explicitly asking for feeders with null hardware ID?
            if (photonFeeder.hardwareId == null && hardwareId == null) {
                return photonFeeder;
            }

            if (photonFeeder.hardwareId != null && photonFeeder.hardwareId.equals(hardwareId)) {
                return photonFeeder;
            }
        }

        return null;
    }

    public static PhotonFeeder findBySlotAddress(int slotAddress) {
        for (Feeder feeder : Configuration.get().getMachine().getFeeders()) {
            if (!(feeder instanceof PhotonFeeder)) {
                continue;
            }

            PhotonFeeder photonFeeder = (PhotonFeeder) feeder;

            if (photonFeeder.slotAddress != null && photonFeeder.slotAddress.equals(slotAddress)) {
                return photonFeeder;
            }
        }

        return null;
    }

    public enum FeederSearchState {
        UNKNOWN,
        SEARCHING,
        FOUND,
        MISSING
    }

    public interface FeederSearchProgressConsumer {
        void accept(int feederAddress, FeederSearchState feederSearchState);
    }

    public static void findAllFeeders(FeederSearchProgressConsumer progressUpdate) throws Exception {
        Logger.info("Searching for Photon Feeders");
        Machine machine = Configuration.get().getMachine();
        PhotonProperties photonProperties = new PhotonProperties(machine);
        int maxFeederAddress = photonProperties.getMaxFeederAddress();
        Logger.debug("Max Photon feeder address: " + maxFeederAddress);

        List<PhotonFeeder> feedersToAdd = new ArrayList<>();

        for (int address = 1; address <= maxFeederAddress; address++) {
            Logger.debug("Querying Photon feeder address: " + address);

            if (progressUpdate != null) {
                progressUpdate.accept(address, FeederSearchState.SEARCHING);
            }

            GetFeederId getFeederId = new GetFeederId(address);
            GetFeederId.Response response = getFeederId.send(photonBus);

            if (progressUpdate != null) {
                progressUpdate.accept(address, response == null ? FeederSearchState.MISSING : FeederSearchState.FOUND);
            }

            if (response == null) {
                PhotonFeeder otherFeeder = findBySlotAddress(address);
                if (otherFeeder != null) {
                    otherFeeder.slotAddress = null;
                    otherFeeder.initialized = false;
                }
            } else {
                PhotonFeeder otherFeeder = findByHardwareId(response.uuid);
                if (otherFeeder == null) {
                    // Try to find an existing feeder without a hardware id before making a new one
                    otherFeeder = findByHardwareId(null);
                    if (otherFeeder == null) {
                        otherFeeder = new PhotonFeeder();
                        feedersToAdd.add(otherFeeder);
                    }
                }

                otherFeeder.setHardwareId(response.uuid);
                otherFeeder.setSlotAddress(address);

                Logger.trace("Found feeder with hardware uuid " + otherFeeder.getHardwareId() + " at address " + otherFeeder.getSlotAddress());
            }
        }

        for (PhotonFeeder feeder : feedersToAdd) {
            Configuration.get().getMachine().addFeeder(feeder);
        }
    }

    @Override
    public boolean canTakeBackPart() {
        return getFeedOptions() == FeedOptions.Normal;
    }

    @Override
    public void takeBackPart(Nozzle nozzle) throws Exception {
        super.takeBackPart(nozzle);
        putPartBack(nozzle);
        setFeedOptions(FeedOptions.SkipNext);
    }

    @Override
    public boolean supportsFeedOptions() {
        return true;
    }
}