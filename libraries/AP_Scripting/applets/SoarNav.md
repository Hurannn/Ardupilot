# SoarNav

This script implements an advanced thermal hunting system for gliders.
When activated, SoarNav takes control of the aircraft's navigation
in soaring-capable flight modes (like Cruise or FBWB) to
autonomously search for and exploit thermals, maximizing flight time.

The script operates using a dual-strategy approach. It divides the
designated flight area into a virtual grid for systematic exploration
and simultaneously builds a memory of previously encountered thermals
("hotspots"). Its decision-making logic is energy-aware: at high
altitudes, it balances exploring new areas with revisiting known
hotspots, while at low altitudes, it prioritizes navigating to the
strongest known thermal to regain energy safely.

SoarNav also features intuitive pilot interaction, allowing for
temporary or persistent manual override via stick gestures, and the
ability to dynamically re-center the search area in-flight.

# Parameters

The script adds the following SNAV_* parameters to control its
behavior.

## SNAV_ENABLE

This must be set to 1 to enable the script. Set to 0 to disable it
completely.

## SNAV_LOG_LVL

Sets the verbosity level of messages sent to the Ground Control Station.
A value of 0 (Silent) reports only critical script errors. A value of 1
(Events) is the default and reports key events like script start/stop,
waypoint reached, new waypoint generation, and thermal memory
recording; this is ideal for standard use. A value of 2 (Detailed Status)
includes all Level 1 events plus a periodic status report with detailed
navigation, grid, and thermal memory data for in-depth debugging.

## SNAV_MAX_DIST

Defines the radius in meters of the circular flight area. The center
is Home by default but can be re-centered in-flight with a stick
gesture. If this parameter is set to 0, the script will switch to
Polygon Mode and attempt to load a boundary file from the SD card.

## SNAV_ROLL_LIMIT

The maximum roll (bank) angle, in degrees, that the script will
command during autonomous navigation. The default is 30 degrees.

## SNAV_WP_RADIUS

The acceptance radius in meters. Defines the distance to a waypoint
at which it is considered 'reached' and a new target is generated.
The default is 30 meters.

## SNAV_NAV_P / SNAV_NAV_D

These are the gains for the script's internal PD (Proportional-Derivative)
navigation controller, which commands roll based on heading error.
SNAV_NAV_P is the proportional gain; higher values result in a more
aggressive response to heading errors. The default is 0.6.
SNAV_NAV_D is the derivative gain; this helps to dampen the
response for smoother control and prevent overshoot. The default is 0.05.

## SNAV_TMEM_ENABLE

Enables (1) or disables (0) the Thermal Memory feature. When enabled,
the script will remember and return to previously found lift areas.

## SNAV_TMEM_CHANCE

The base probability, as a percentage (0-100), that the script will
choose to navigate towards a known thermal instead of exploring a new
grid cell. This chance is dynamically adjusted by the script based on
recent success in finding thermals. Default is 50%.

## SNAV_TMEM_LIFE

The lifetime in seconds of a 'hotspot' (a stored thermal). After
this time, the point is considered expired and is removed from memory.
The default is 1200 seconds (20 minutes).

## SNAV_WIND_COMP

A compensation factor used to estimate the upwind position of a
thermal relative to where it was detected. This improves the accuracy
of the thermal memory by accounting for wind drift. The default is 60.

# Operation

## Installation and Setup

    Place the SoarNav.lua script into the APM/SCRIPTS directory
    on the flight controller's microSD card.

    Set SCR_ENABLE to 1 and reboot the flight controller.

    Refresh parameters in your ground station. The SNAV_* parameters
    should now be visible.

    Set SNAV_ENABLE to 1.

    Configure an RC channel switch to activate the Soaring
    feature. This is done by setting an RCx_OPTION to
    88 (Soaring Active). This switch will be used to turn the main
    ArduPilot SOAR feature on and off. SoarNav will only run when this
    feature is active.

## Area Configuration

You must choose one of two methods to define the operational area.

Radius Mode is enabled by setting SNAV_MAX_DIST to your desired
search radius in meters (e.g., 500). The script will operate within this
circle.
Polygon Mode is enabled by setting SNAV_MAX_DIST to 0. This requires
a file named snav.poly in the root directory of the microSD card.
This flight area file can be easily generated and saved using the
Mission Planner interface. The file should contain a list of GPS
coordinates (latitude longitude, space-separated) defining the
vertices of your flight area, one vertex per line. The script will
automatically close the polygon.

## In-Flight Use

    Take off and enter a compatible flight mode (Cruise or FBWB).

    Ensure you are within the configured flight area and above the
    SOAR_ALT_MIN altitude.

    Activate the main ArduPilot Soaring feature using the switch
    you configured with RCx_OPTION = 88.

    When SoarNav is active, it will begin generating waypoints and
    navigating autonomously. You will see "SoarNav" messages on your GCS.

    To pause SoarNav and regain manual control, simply disable the
    Soaring feature with your switch. Re-enabling it will resume
    autonomous navigation.

# Pilot Interaction and Stick Gestures

SoarNav is designed to be interactive, allowing the pilot to override
its behavior without changing flight modes.

## Temporary Override

Any input on the pitch or yaw sticks will immediately pause
SoarNav's navigation commands. The script will automatically resume
navigation a few seconds after the sticks are returned to center.

## Persistent Manual Override (Roll Gesture)

A rapid sequence of full left-right-left-right roll stick
movements toggles a persistent manual override. This allows the pilot
to fly freely without the script resuming automatically. Repeat the
gesture to deactivate the override and return control to SoarNav.
GCS messages will confirm the state change.

## Area Re-Centering (Pitch Gesture)

If using Radius Mode, a rapid sequence of full
up-down-up-down-up pitch stick movements will re-center the
circular search area to the aircraft's current location. This is useful
for shifting the focus of exploration during a long flight. This
gesture is only active when you have temporary stick override.