--[[
SoarNav: an advanced feature for hunting thermals by Marco Robustini.
-- Version 0.9.2 - 2025/07/29

In early versions of SOAR mode in Ardupilot, which I collaborated on with 
the author, when the glider exited THERMAL mode the heading pointed 
approximately to home. This feature that I wanted was later removed 
because it was considered unnecessary by some. I therefore created a LUA
that reimplements it but in an advanced way.
By
activating SoarNav when the autopilot has SOAR active and Cruise or 
FBWB is used as the flight mode the script continuously monitors a 
predefined radius from the home or a custom polygon area. But it doesn't
just fly randomly. SoarNav uses a sophisticated set of strategies to 
maximize flight time.

================================================================================
Key Features
================================================================================

- **Systematic Exploration (Virtual Grid)**:
  The script divides the flight area into a virtual grid and systematically
  explores it. It generates new waypoints by choosing primarily from cells
  that have not yet been visited. Once all valid cells have been explored,
  the grid's visited status is reset, and the exploration cycle begins anew.

- **Advanced Thermal Analysis (Strength & Quality)**:
  The script doesn't just remember *where* a thermal was, but also *how good*
  it was. It analyzes the climb rate during thermal mode to save its average
  and peak strength, and also assesses its **consistency** (steady vs. variable lift).
  
- **Intelligent Thermal Memory with Boundary Checks**:
  When the flight controller enters THERMAL mode, the script saves a "hotspot"
  corrected for wind drift. It intelligently **ignores and discards thermals**
  detected outside the defined operational area (polygon or radius). Wind
  compensation uses an adaptive factor based on the thermal's measured strength,
  enhancing the accuracy of return-to-thermal navigation.

- **Energy-Aware & Dual-Strategy Decision Making**:
  The script's core logic is based on energy management.
    - **Low Energy**: When low on altitude, it prioritizes safety by navigating
      deterministically to the **strongest known thermal** to regain height.
    - **Normal Energy**: With ample altitude, it uses a probabilistic "tournament"
      selection, picking two random hotspots and targeting the stronger of the pair.
      This balances exploiting known lift with re-evaluating other good areas.

- **Adaptive Strategy (Dynamic Memory Chance)**:
  The script dynamically adjusts its strategy. Based on the number of recent
  thermals found, it increases or decreases the probability of returning to a
  known hotspot versus exploring a new, unknown grid cell.

- **Dynamic Thermal Drift Prediction**:
  When targeting a known hotspot, the script uses the current wind vector and
  the hotspot's age to predict its new, drifted position, increasing the
  chances of a successful intercept.

- **Safety Pre-flight Checks (Parameter Validation)**:
  Before starting, the script validates key SNAV_* and SOAR_* parameters to
  ensure they are within safe and logical ranges, preventing errors from
  misconfiguration and disabling itself if an unsafe value is detected.
  
- **Tactical Anti-Stall Navigation (Upwind Repositioning)**:
  If the script detects that the aircraft is not making progress towards a
  waypoint due to wind, it initiates an automatic maneuver. It temporarily flies
  to an upwind position to then approach the original target from a more
  favorable trajectory.

- **Adaptive Waypoint Timeout**:
  The script intelligently adjusts the maximum time allowed to reach a waypoint
  based on current wind conditions, allowing more time in strong headwinds.

- **Intuitive Pilot Override & Stick Gestures**:
  - **Temporary Override**: Moving the Pitch or Yaw sticks instantly pauses the
    script's autonomous navigation. It resumes a few seconds after sticks are centered.
  - **Persistent Override (Roll Gesture)**: A rapid sequence of roll stick movements
    toggles a persistent manual override, allowing the pilot to fly freely until the
    gesture is repeated.
  - **Dynamic Area Re-centering (Pitch Gesture)**: A rapid sequence of pitch stick
    movements re-centers the circular search area to the aircraft's current location.

- **Dual Area System (Radius or Polygon)**:
  The search area can be defined as a circular radius from a dynamically chosen
  center point or a custom polygon loaded from the SD card.

- **Advanced Logging System**:
  A multi-level logging system (controlled by SNAV_LOG_LVL) provides clear
  operational feedback, from key events to detailed real-time status for debugging.

  Example of a Level 2 status message sent to the GCS:

  27/07/2025 07:31:37 : SoarNav: Thermal mem: 0 active
  27/07/2025 07:31:37 : SoarNav: Grid: 7/116 expl. 6% | Curr. Cell: 157
  27/07/2025 07:31:37 : SoarNav: WP: D:263m, Hdg Err:+0, Roll:+0.2
  27/07/2025 07:31:37 : SoarNav: Nav to: Cell: 173


--------------------------------------------------------------------------------
Script Parameters (SNAV_*)
--------------------------------------------------------------------------------
- SNAV_ENABLE: Master switch to enable (1) or disable (0) the script.
- SNAV_LOG_LVL: Sets the verbosity level of messages sent to the Ground Control Station.
    - **0 (Silent)**: Only critical script errors are reported.
    - **1 (Events)**: [Default] Reports key events: script start/stop, waypoint
      reached, new waypoint generation, and thermal memory recording. Ideal for standard use.
    - **2 (Detailed Status)**: Includes all Level 1 events PLUS a periodic
      status report for in-depth, real-time debugging.
- SNAV_MAX_DIST: Defines the radius in meters of the circular flight area around the home
  point. If set to 0, the script will look for a polygon file (snav.poly) on the SD card.
- SNAV_ROLL_LIMIT: The maximum roll (bank) angle, in degrees, that the script will
  command during autonomous navigation.
- SNAV_WP_RADIUS: The acceptance radius in meters. Defines the distance to a waypoint
  at which it is considered 'reached'.
- SNAV_NAV_P / SNAV_NAV_D: Gains for the PD (Proportional-Derivative) navigation controller.
  SNAV_NAV_P controls responsiveness, while SNAV_NAV_D dampens the response for smoother control.
- SNAV_TMEM_ENABLE: Enables (1) or disables (0) the Thermal Memory feature, which allows
  the script to remember and return to found lift areas.
- SNAV_TMEM_CHANCE: The base probability, as a percentage (0-100), that the script will
  choose to navigate towards a known thermal instead of exploring a new grid cell.
- SNAV_TMEM_LIFE: The lifetime in seconds of a 'hotspot' (a stored thermal). After this
  time, the point is considered expired and is removed from memory.
- SNAV_WIND_COMP: A compensation factor used to estimate the upwind position of a thermal
  relative to where it was detected, improving memory accuracy.
]]

--[[ SCRIPT INITIALIZATION ]]--

math.randomseed(tonumber(tostring(millis() or 0)) or 0)

-- Enums for GCS message severity and script internal state
local MAV_SEVERITY = {
    EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3,
    WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7
}

local SCRIPT_STATE = {
    IDLE = 0,
    NAVIGATING = 1,
    PILOT_OVERRIDE = 2,
    THERMAL_PAUSE = 3,
    ERROR = 4
}

--[[ PARAMETER DEFINITION AND BINDING ]]--

local PARAM_TABLE_KEY = 151
local PARAM_PREFIX = "SNAV_"
local param_list = {
    {name = "ENABLE",      default = 0, description = "Enable script (0=Disable, 1=Enable)"},
    {name = "LOG_LVL",     default = 1, description = "GCS log verbosity (0=Silent, 1=Normal, 2=Detailed)"},
    {name = "MAX_DIST",    default = 500, description = "Maximum distance (meters) from home or 0 for polygon area"},
    {name = "ROLL_LIMIT",  default = 30, description = "Maximum roll angle (degrees) for navigation"},
    {name = "WP_RADIUS",   default = 30, description = "Acceptance radius for virtual waypoints (meters)"},
    {name = "NAV_P",       default = 0.6, description = "Navigation P-gain for roll controller (higher is more responsive)"},
    {name = "NAV_D",       default = 0.05, description = "Navigation D-gain for roll controller (dampens response)"},
    {name = "TMEM_ENABLE", default = 1, description = "Enable Thermal Memory (0=Disable, 1=Enable)"},
    {name = "TMEM_CHANCE", default = 50, description = "Chance to use thermal memory (percent, 0-100)"},
    {name = "TMEM_LIFE",   default = 1200, description = "Lifetime of a thermal hotspot (seconds)"},
    {name = "WIND_COMP",   default = 60, description = "Upwind compensation factor for thermal memory"}
}

-- Creates the script's parameters in the ArduPilot parameter list
local function add_params()
    assert(param:add_table(PARAM_TABLE_KEY, PARAM_PREFIX, #param_list),
           string.format("CRITICAL ERROR: SoarNav - Could not add param table '%s'.", PARAM_PREFIX))
    for i, p in ipairs(param_list) do
        assert(param:add_param(PARAM_TABLE_KEY, i, p.name, p.default),
               string.format("CRITICAL ERROR: SoarNav - Could not add param %s%s.", PARAM_PREFIX, p.name))
    end
end

if not param:get(PARAM_PREFIX .. "ENABLE") then
    add_params()
end

-- Binds a script variable to an ArduPilot parameter for efficient access
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('CRITICAL ERROR: SoarNav - Could not find parameter %s.', name))
    return p
end

-- Binding all script and relevant ArduPilot parameters
local p_enable        = bind_param(PARAM_PREFIX .. "ENABLE")
local p_log_lvl       = bind_param(PARAM_PREFIX .. "LOG_LVL")
local p_max_dist      = bind_param(PARAM_PREFIX .. "MAX_DIST")
local p_roll_limit    = bind_param(PARAM_PREFIX .. "ROLL_LIMIT")
local p_wp_radius     = bind_param(PARAM_PREFIX .. "WP_RADIUS")
local p_nav_p         = bind_param(PARAM_PREFIX .. "NAV_P")
local p_nav_d         = bind_param(PARAM_PREFIX .. "NAV_D")
local p_tmem_enable   = bind_param(PARAM_PREFIX .. "TMEM_ENABLE")
local p_tmem_chance   = bind_param(PARAM_PREFIX .. "TMEM_CHANCE")
local p_tmem_life     = bind_param(PARAM_PREFIX .. "TMEM_LIFE")
local p_wind_comp     = bind_param(PARAM_PREFIX .. "WIND_COMP")
local p_soar_alt_min  = bind_param("SOAR_ALT_MIN")
local p_sys_roll_limit  = bind_param("ROLL_LIMIT_DEG")
local p_soar_alt_max = bind_param("SOAR_ALT_MAX")

--[[ CONSTANTS AND GLOBAL STATE ]]--

-- Fixed values for script operation and tuning (grouped by function)
local SoarNavConstants = {
    -- Timing Constants
    pilot_resume_delay_ms = 5000,
    wp_timeout_ms = 300000,
    grid_update_interval_ms = 2000,
    rc_search_interval_ms = 5000,
    thermal_history_window_ms = 900000,
    
    -- Anti-Stall Logic
    STUCK_GRACE_PERIOD_MS = 30000,
    STUCK_PROGRESS_CHECK_INTERVAL_MS = 20000,
    STUCK_MIN_PROGRESS_M = 10,

    -- Navigation and Control
    roll_smoothing_factor = 0.1,
    HYSTERESIS_MARGIN = 5,

    -- Area and Grid Management
    polygon_filename = "snav.poly",
    min_cell_size_m = 30,
    max_total_grid_cells = 128,
    grid_init_cells_per_call = 20,
    MAX_GRID_SEARCH_ATTEMPTS_PER_CALL = 10,

    -- Thermal Logic
    max_hotspots = 10,
    THERMAL_CONSISTENCY_VARIANCE_THRESHOLD = 0.5,
    MAX_DRIFT_AGE_S = 600,
    MAX_DRIFT_FACTOR = 1.5,
    
    -- Pilot Gestures
    rc_deadzone = 30,
    PITCH_GESTURE_COUNT_TARGET = 4,
    PITCH_GESTURE_THRESHOLD = 0.5,
    PITCH_GESTURE_TIMEOUT_MS = 2000,
    ROLL_GESTURE_COUNT_TARGET = 4,
    ROLL_GESTURE_THRESHOLD = 0.5,
    ROLL_GESTURE_TIMEOUT_MS = 2000,

    -- ArduPilot-specific Constants
    rc_thresh_high = 1500,
    rc_opt_soaring_active = 88,
    mode_fbwb = 6,
    mode_cruise = 7,
    mode_thermal = 24,

    -- System Constants
    max_location_errors = 10
}

-- Table to hold all dynamic script state variables
local SoarNavGlobals = {
    script_state = SCRIPT_STATE.IDLE,
    detected_soaring_rc_channel = nil,
    rc_roll_channel = rc:get_channel(1),
    last_rc_search_ms = 0,
    polygon_points = {},
    polygon_bounds = nil,
    last_cell_index = nil,
    use_polygon_area = false,
    target_lat = nil,
    target_lon = nil,
    waypoint_start_time_ms = 0,
    last_pilot_input_ms = 0,
    last_heading_error = 0,
    last_commanded_roll_deg = 0,
    thermal_hotspots = {},
    location_error_count = 0,
    grid_cells = {},
    grid_rows = 0,
    grid_cols = 0,
    grid_bounds = nil,
    grid_initialized = false,
    is_initializing = false,
    grid_init_step = 0,
    grid_init_row = 1,
    grid_init_col = 1,
    grid_populate_index = 1,
    last_grid_update_ms = 0,
    rc1_min = 1000, rc1_max = 2000, rc1_trim = 1500,
    rc2_min = 1000, rc2_max = 2000, rc2_trim = 1500,
    rc4_min = 1000, rc4_max = 2000, rc4_trim = 1500,
    rc_limits_read = false,
    thermal_entry_timestamps = {},
    g_waypoint_source_info = "N/A",
    energy_state = "NORMAL",
    was_in_thermal_mode = false,
    is_monitoring_thermal = false,
    last_thermal_sample_ms = 0,
    current_thermal_stats = {},
    last_used_hotspot_timestamp = 0,
    waypoint_search_in_progress = false,
    logged_ignore_message = false,
    last_snav_max_dist_value = -1,
    valid_cell_indices = {},
    unvisited_cell_indices = {},
    distance_to_wp = -1,
    is_repositioning = false,
    original_target_lat = nil,
    original_target_lon = nil,
    last_progress_check_ms = 0,
    distance_at_last_check = -1,
    stuck_counter = 0,
    last_grid_reset_ms = 0,
    dynamic_center_location = nil,
    pitch_gesture_state = "idle",
    pitch_gesture_count = 0,
    pitch_gesture_triggered_this_override = false,
    pitch_gesture_start_ms = 0,
    roll_gesture_state = "idle",
    roll_gesture_count = 0,
    roll_gesture_start_ms = 0,
    manual_override_active = false,
    force_grid_after_reset = false,
    cached_wind = nil,
    wind_cache_time = 0
}


--[[ UTILITY AND HELPER FUNCTIONS ]]--

-- Safe decrement and getters to prevent errors
local function safe_decrement(n)
    return math.max(0, n - 1)
end

local function safe_get(p, fallback)
    local ok, val = pcall(function() return p:get() end)
    if ok and val ~= nil then return val else return fallback end
end

-- GCS logging wrapper with verbosity check
local function log_gcs(severity, level, message)
    if (safe_get(p_log_lvl, 1)) >= level then
        gcs:send_text(severity, "SoarNav: " .. message)
    end
end

-- Caches wind data to reduce AHRS requests
local function get_wind_vector()
    if not SoarNavGlobals.cached_wind or (tonumber(tostring(millis())) - tonumber(tostring(SoarNavGlobals.wind_cache_time))) > 5000 then
        SoarNavGlobals.cached_wind = ahrs:wind_estimate()
        SoarNavGlobals.wind_cache_time = millis()
    end
    return SoarNavGlobals.cached_wind
end

-- Safe atan2 implementation to avoid errors with zero inputs
local function atan2_safe(y, x)
    if x > 0 then return math.atan(y / x)
    elseif x < 0 and y >= 0 then return math.atan(y / x) + math.pi
    elseif x < 0 and y < 0 then return math.atan(y / x) - math.pi
    elseif x == 0 and y > 0 then return math.pi / 2
    elseif x == 0 and y < 0 then return -math.pi / 2
    else return 0 end
end

-- Pre-flight check to validate that parameters are in safe ranges
local function validate_params()
    local all_valid = true
    local max_dist = safe_get(p_max_dist, 500)
    if max_dist < 0 then
        log_gcs(MAV_SEVERITY.ERROR, 0, "SNAV_MAX_DIST cannot be negative.")
        all_valid = false
    end
    local roll_limit = safe_get(p_roll_limit, 30)
    if roll_limit < 10 or roll_limit > 50 then
        log_gcs(MAV_SEVERITY.WARNING, 1, "SNAV_ROLL_LIMIT is outside recommended range (10-50).")
    end
    local tmem_chance = safe_get(p_tmem_chance, 50)
    if tmem_chance < 0 or tmem_chance > 100 then
        log_gcs(MAV_SEVERITY.ERROR, 0, "SNAV_TMEM_CHANCE must be between 0 and 100.")
        all_valid = false
    end
    return all_valid
end

-- Grid and energy state helpers
local function get_cell_index_from_location(lat, lon)
    if not SoarNavGlobals.grid_initialized or not SoarNavGlobals.grid_bounds then return nil end
    if lat < SoarNavGlobals.grid_bounds.min_lat or lat > SoarNavGlobals.grid_bounds.max_lat or lon < SoarNavGlobals.grid_bounds.min_lon or lon > SoarNavGlobals.grid_bounds.max_lon then
        return nil
    end
    local lat_fraction = (lat - SoarNavGlobals.grid_bounds.min_lat) / (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat)
    local lon_fraction = (lon - SoarNavGlobals.grid_bounds.min_lon) / (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon)
    local row = math.floor(lat_fraction * SoarNavGlobals.grid_rows) + 1
    local col = math.floor(lon_fraction * SoarNavGlobals.grid_cols) + 1
    row = math.max(1, math.min(SoarNavGlobals.grid_rows, row))
    col = math.max(1, math.min(SoarNavGlobals.grid_cols, col))
    return (row - 1) * SoarNavGlobals.grid_cols + col
end

local function assess_energy_state()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    if not dist_from_home_3d then return "UNKNOWN" end
    local current_alt = -dist_from_home_3d:z()
    local min_alt = safe_get(p_soar_alt_min, 40)
    local max_alt = safe_get(p_soar_alt_max, 200)
    local margin = (max_alt - min_alt) * 0.3
    
    local lower_threshold = min_alt + margin - SoarNavConstants.HYSTERESIS_MARGIN
    local upper_threshold = min_alt + margin + SoarNavConstants.HYSTERESIS_MARGIN

    if current_alt < min_alt then
        SoarNavGlobals.energy_state = "CRITICAL"
    elseif SoarNavGlobals.energy_state == "NORMAL" and current_alt < lower_threshold then
        SoarNavGlobals.energy_state = "LOW"
    elseif SoarNavGlobals.energy_state == "LOW" and current_alt > upper_threshold then
        SoarNavGlobals.energy_state = "NORMAL"
    elseif SoarNavGlobals.energy_state ~= "CRITICAL" and current_alt >= lower_threshold and SoarNavGlobals.energy_state ~= "LOW" then
        SoarNavGlobals.energy_state = "NORMAL"
    end
    
    return SoarNavGlobals.energy_state
end

local function log_thermal_event(hotspot)
    local wind = get_wind_vector()
    if wind then
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal@%.5f,%.5f: %.1fm/s (max %.1fm/s, %s) Wind:%.1fm/s@%d", hotspot.lat, hotspot.lon, hotspot.avg_strength, hotspot.max_strength, hotspot.consistency, wind:length(), math.floor((math.deg(atan2_safe(wind:y(), wind:x())) + 360) % 360)))
    else
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("Thermal@%.5f,%.5f: %.1fm/s (max %.1fm/s, %s) Wind:N/A", hotspot.lat, hotspot.lon, hotspot.avg_strength, hotspot.max_strength, hotspot.consistency))
    end
end

local function calculate_thermal_variance(samples)
    if not samples or #samples < 2 then return 0 end
    local sum = 0
    for _, val in ipairs(samples) do sum = sum + val end
    local mean = sum / #samples
    local variance_sum = 0
    for _, val in ipairs(samples) do variance_sum = variance_sum + (val - mean)^2 end
    return variance_sum / #samples
end

--[[ STATE MANAGEMENT AND GEOSPATIAL FUNCTIONS ]]--

-- Manages the main state machine transitions
local function set_script_state(new_state, reason)
    if SoarNavGlobals.script_state ~= new_state then
        if new_state == SCRIPT_STATE.IDLE and SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
            SoarNavGlobals.pitch_gesture_state = "idle"
            SoarNavGlobals.pitch_gesture_count = 0
            SoarNavGlobals.pitch_gesture_start_ms = 0
            SoarNavGlobals.pitch_gesture_triggered_this_override = false
            SoarNavGlobals.roll_gesture_state = "idle"
            SoarNavGlobals.roll_gesture_count = 0
            SoarNavGlobals.roll_gesture_start_ms = 0
            SoarNavGlobals.manual_override_active = false
        end
        SoarNavGlobals.script_state = new_state
        log_gcs(MAV_SEVERITY.NOTICE, 1, reason)
        if new_state == SCRIPT_STATE.IDLE or new_state == SCRIPT_STATE.PILOT_OVERRIDE or new_state == SCRIPT_STATE.THERMAL_PAUSE or new_state == SCRIPT_STATE.ERROR then
            if SoarNavGlobals.rc_roll_channel then
                SoarNavGlobals.rc_roll_channel:set_override(0)
            end
            SoarNavGlobals.target_lat, SoarNavGlobals.target_lon = nil, nil
            SoarNavGlobals.last_heading_error = 0
            SoarNavGlobals.last_commanded_roll_deg = 0
        end
    end
end

-- Standard geo-spatial calculations (distance, bearing, offset)
local function haversine(lat1, lon1, lat2, lon2)
    local R = 6371000
    local dLat = math.rad(lat2 - lat1)
    local dLon = math.rad(lon2 - lon1)
    local a = math.sin(dLat/2)^2 + math.cos(math.rad(lat1)) * math.cos(math.rad(lat2)) * math.sin(dLon/2)^2
    local c = 2 * atan2_safe(math.sqrt(a), math.sqrt(1 - a))
    return R * c
end

local function calculate_bearing(lat1, lon1, lat2, lon2)
    local dLon = math.rad(lon2 - lon1)
    local y = math.sin(dLon) * math.cos(math.rad(lat2))
    local x = math.cos(math.rad(lat1)) * math.sin(math.rad(lat2)) -
              math.sin(math.rad(lat1)) * math.cos(math.rad(lat2)) * math.cos(dLon)
    local brng = math.deg(atan2_safe(y, x))
    return (brng + 360) % 360
end

local function location_offset(lat, lon, brng, dist)
    local R = 6371000
    local lat_rad, lon_rad = math.rad(lat), math.rad(lon)
    local brng_rad = math.rad(brng)
    local new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(dist / R) + math.cos(lat_rad) * math.sin(dist / R) * math.cos(brng_rad))
    local new_lon_rad = lon_rad + atan2_safe(math.sin(brng_rad) * math.sin(dist / R) * math.cos(lat_rad), math.cos(dist / R) - math.sin(lat_rad) * math.sin(new_lat_rad))
    local new_loc = Location()
    new_loc:lat(math.floor(math.deg(new_lat_rad) * 1e7 + 0.5))
    new_loc:lng(math.floor(math.deg(new_lon_rad) * 1e7 + 0.5))
    return new_loc
end

-- Point-in-polygon check algorithm
local function is_point_in_polygon(lat, lon, poly)
    if not poly or #poly < 3 or not SoarNavGlobals.polygon_bounds then return false end
    if lat < SoarNavGlobals.polygon_bounds.min_lat or lat > SoarNavGlobals.polygon_bounds.max_lat or lon < SoarNavGlobals.grid_bounds.min_lon or lon > SoarNavGlobals.grid_bounds.max_lon then
        return false
    end
    local is_inside = false
    local j = #poly
     for i = 1, #poly do
        if poly[i] and poly[j] and poly[i].lat and poly[i].lon and poly[j].lat and poly[j].lon then
            local vy_i, vx_i = poly[i].lat, poly[i].lon
            local vy_j, vx_j = poly[j].lat, poly[j].lon
            if vy_i ~= vy_j then
                if ((vy_i > lat) ~= (vy_j > lat)) and (lon < (vx_j - vx_i) * (lat - vy_i) / (vy_j - vy_i) + vx_i) then
                    is_inside = not is_inside
                end
            end
        end
        j = i
    end
    return is_inside
end


local function get_active_center_location()
    if SoarNavGlobals.dynamic_center_location then
        return SoarNavGlobals.dynamic_center_location
    else
        return ahrs:get_home()
    end
end

--[[ PILOT GESTURE DETECTION ]]--

-- Detects pitch stick gestures for re-centering the search area
local function check_pitch_gesture()
    if (safe_get(p_max_dist, 0)) <= 0 or SoarNavGlobals.pitch_gesture_triggered_this_override then
        return
    end

    if SoarNavGlobals.pitch_gesture_state ~= "idle" and (tonumber(tostring(millis())) - tonumber(tostring(SoarNavGlobals.pitch_gesture_start_ms))) > SoarNavConstants.PITCH_GESTURE_TIMEOUT_MS then
        SoarNavGlobals.pitch_gesture_state = "idle"
        SoarNavGlobals.pitch_gesture_count = 0
    end

    local pitch_pwm = rc:get_pwm(2)
    if not pitch_pwm then return end

    local normalized_pitch
    if pitch_pwm > SoarNavGlobals.rc2_trim then
        normalized_pitch = (pitch_pwm - SoarNavGlobals.rc2_trim) / (SoarNavGlobals.rc2_max - SoarNavGlobals.rc2_trim)
    else
        normalized_pitch = (pitch_pwm - SoarNavGlobals.rc2_trim) / (SoarNavGlobals.rc2_trim - SoarNavGlobals.rc2_min)
    end
    
    if SoarNavGlobals.pitch_gesture_state == "idle" then
        if math.abs(normalized_pitch) > SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_start_ms = millis()
            SoarNavGlobals.pitch_gesture_count = 1
            if normalized_pitch > 0 then
                SoarNavGlobals.pitch_gesture_state = "waiting_for_down"
            else
                SoarNavGlobals.pitch_gesture_state = "waiting_for_up"
            end
        end
    elseif SoarNavGlobals.pitch_gesture_state == "waiting_for_down" then
        if normalized_pitch < -SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_count = SoarNavGlobals.pitch_gesture_count + 1
            SoarNavGlobals.pitch_gesture_state = "waiting_for_up"
        end
    elseif SoarNavGlobals.pitch_gesture_state == "waiting_for_up" then
        if normalized_pitch > SoarNavConstants.PITCH_GESTURE_THRESHOLD then
            SoarNavGlobals.pitch_gesture_count = SoarNavGlobals.pitch_gesture_count + 1
            SoarNavGlobals.pitch_gesture_state = "waiting_for_down"
        end
    end

    if SoarNavGlobals.pitch_gesture_count >= SoarNavConstants.PITCH_GESTURE_COUNT_TARGET then
        log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: SoarNav area re-centered.")
        SoarNavGlobals.dynamic_center_location = ahrs:get_location()
        SoarNavGlobals.target_lat = nil
        SoarNavGlobals.grid_initialized = false
        SoarNavGlobals.pitch_gesture_triggered_this_override = true
    end
end

-- Detects roll stick gestures for toggling persistent manual override
local function check_roll_gesture()
    if SoarNavGlobals.roll_gesture_state ~= "idle" and (tonumber(tostring(millis())) - tonumber(tostring(SoarNavGlobals.roll_gesture_start_ms))) > SoarNavConstants.ROLL_GESTURE_TIMEOUT_MS then
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
    end

    local roll_pwm = rc:get_pwm(1)
    if not roll_pwm then return end

    local normalized_roll
    if roll_pwm > SoarNavGlobals.rc1_trim then
        normalized_roll = (roll_pwm - SoarNavGlobals.rc1_trim) / (SoarNavGlobals.rc1_max - SoarNavGlobals.rc1_trim)
    else
        normalized_roll = (roll_pwm - SoarNavGlobals.rc1_trim) / (SoarNavGlobals.rc1_trim - SoarNavGlobals.rc1_min)
    end
    
    if SoarNavGlobals.roll_gesture_state == "idle" then
        if math.abs(normalized_roll) > SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_start_ms = millis()
            SoarNavGlobals.roll_gesture_count = 1
            if normalized_roll > 0 then
                SoarNavGlobals.roll_gesture_state = "waiting_for_left"
            else
                SoarNavGlobals.roll_gesture_state = "waiting_for_right"
            end
        end
    elseif SoarNavGlobals.roll_gesture_state == "waiting_for_left" then
        if normalized_roll < -SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_count = SoarNavGlobals.roll_gesture_count + 1
            SoarNavGlobals.roll_gesture_state = "waiting_for_right"
        end
    elseif SoarNavGlobals.roll_gesture_state == "waiting_for_right" then
        if normalized_roll > SoarNavConstants.ROLL_GESTURE_THRESHOLD then
            SoarNavGlobals.roll_gesture_count = SoarNavGlobals.roll_gesture_count + 1
            SoarNavGlobals.roll_gesture_state = "waiting_for_left"
        end
    end

    if SoarNavGlobals.roll_gesture_count >= SoarNavConstants.ROLL_GESTURE_COUNT_TARGET then
        SoarNavGlobals.manual_override_active = not SoarNavGlobals.manual_override_active
        if SoarNavGlobals.manual_override_active then
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: Manual override activated (persistent).")
        else
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Stick CMD: Manual override deactivated. Script will resume if sticks centered.")
        end
        SoarNavGlobals.roll_gesture_state = "idle"
        SoarNavGlobals.roll_gesture_count = 0
        SoarNavGlobals.roll_gesture_start_ms = 0
    end
end

--[[ EXPLORATION GRID SYSTEM ]]--

-- Kicks off the non-blocking grid initialization
local function initialize_grid()
    SoarNavGlobals.is_initializing = true
    SoarNavGlobals.grid_initialized = false
    SoarNavGlobals.grid_init_step = 1
    SoarNavGlobals.grid_cells = {}
    SoarNavGlobals.valid_cell_indices = {}
    SoarNavGlobals.unvisited_cell_indices = {}
    SoarNavGlobals.last_cell_index = nil
    log_gcs(MAV_SEVERITY.INFO, 1, "Starting grid initialization process...")
end

-- Manages the step-by-step creation of the grid to avoid blocking the script
local function manage_grid_initialization()
    if not SoarNavGlobals.is_initializing then return end
    local active_center = get_active_center_location()
    if not active_center then
        log_gcs(MAV_SEVERITY.ERROR, 0, "Grid init failed: active center not available.")
        SoarNavGlobals.is_initializing = false
        SoarNavGlobals.grid_init_step = 0
        return
    end
    if SoarNavGlobals.grid_init_step == 1 then
        if SoarNavGlobals.use_polygon_area and SoarNavGlobals.polygon_bounds then
            SoarNavGlobals.grid_bounds = SoarNavGlobals.polygon_bounds
        else
            local max_dist = p_max_dist:get() or 500
            local center_lat, center_lon = active_center:lat()/1e7, active_center:lng()/1e7
            local corner_dist = max_dist * 1.4142
            local sw_corner_loc = location_offset(center_lat, center_lon, 225, corner_dist)
            local ne_corner_loc = location_offset(center_lat, center_lon, 45, corner_dist)
            SoarNavGlobals.grid_bounds = {
                min_lat = sw_corner_loc:lat()/1e7, max_lat = ne_corner_loc:lat()/1e7,
                min_lon = sw_corner_loc:lng()/1e7, max_lon = ne_corner_loc:lng()/1e7
            }
        end
        SoarNavGlobals.grid_init_step = 2
    elseif SoarNavGlobals.grid_init_step == 2 then
        local height_m = haversine(SoarNavGlobals.grid_bounds.min_lat, SoarNavGlobals.grid_bounds.min_lon, SoarNavGlobals.grid_bounds.max_lat, SoarNavGlobals.grid_bounds.min_lon)
        local width_m = haversine(SoarNavGlobals.grid_bounds.min_lat, SoarNavGlobals.grid_bounds.min_lon, SoarNavGlobals.grid_bounds.min_lat, SoarNavGlobals.grid_bounds.max_lon)
        if height_m < 1 or width_m < 1 then
            log_gcs(MAV_SEVERITY.ERROR, 0, "Grid area is too small. Initialization aborted.")
            SoarNavGlobals.is_initializing = false
            SoarNavGlobals.grid_init_step = 0
            return
        end
        local area_m2 = height_m * width_m
        local approx_cell_size_m = math.sqrt(area_m2 / SoarNavConstants.max_total_grid_cells)
        local final_cell_size_m = math.max(SoarNavConstants.min_cell_size_m, approx_cell_size_m)
        SoarNavGlobals.grid_rows = math.max(1, math.floor(height_m / final_cell_size_m))
        SoarNavGlobals.grid_cols = math.max(1, math.floor(width_m / final_cell_size_m))
        SoarNavGlobals.grid_populate_index = 1
        SoarNavGlobals.grid_init_step = 3
    elseif SoarNavGlobals.grid_init_step == 3 then
        local total_cells = SoarNavGlobals.grid_rows * SoarNavGlobals.grid_cols
        local cells_processed = 0
        local cells_per_call = SoarNavConstants.grid_init_cells_per_call
        while SoarNavGlobals.grid_populate_index <= total_cells and cells_processed < cells_per_call do
            table.insert(SoarNavGlobals.grid_cells, {visit_count = 0})
            SoarNavGlobals.grid_populate_index = SoarNavGlobals.grid_populate_index + 1
            cells_processed = cells_processed + 1
        end
        if SoarNavGlobals.grid_populate_index > total_cells then
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Grid: %d rows, %d cols. Scan cells...", SoarNavGlobals.grid_rows, SoarNavGlobals.grid_cols))
            SoarNavGlobals.grid_init_row = 1
            SoarNavGlobals.grid_init_col = 1
            SoarNavGlobals.grid_init_step = 4
        end
    elseif SoarNavGlobals.grid_init_step == 4 then
        local cells_processed = 0
        local max_dist_for_check = p_max_dist:get() or 500
        local max_dist_sq = max_dist_for_check * max_dist_for_check
        local center_lat, center_lon = active_center:lat()/1e7, active_center:lng()/1e7
        local deg_to_rad = math.pi / 180
        local lon_dist_factor = math.cos(center_lat * deg_to_rad) * 111320.0
        
        local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat) / SoarNavGlobals.grid_rows
        local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon) / SoarNavGlobals.grid_cols
        
        while SoarNavGlobals.grid_init_row <= SoarNavGlobals.grid_rows and cells_processed < SoarNavConstants.grid_init_cells_per_call do
            local r = SoarNavGlobals.grid_init_row
            local c = SoarNavGlobals.grid_init_col
            local cell_center_lat = SoarNavGlobals.grid_bounds.min_lat + (r - 0.5) * cell_height_lat
            local cell_center_lon = SoarNavGlobals.grid_bounds.min_lon + (c - 0.5) * cell_width_lon
            local cell_is_valid = false
            if SoarNavGlobals.use_polygon_area then
                if is_point_in_polygon(cell_center_lat, cell_center_lon, SoarNavGlobals.polygon_points) then
                    cell_is_valid = true
                end
            else
                local dy = (cell_center_lat - center_lat) * 111132.9
                local dx = (cell_center_lon - center_lon) * lon_dist_factor
                local dist_sq = dx*dx + dy*dy
                
                if dist_sq <= max_dist_sq then
                    cell_is_valid = true
                end
            end
            
            if cell_is_valid then
                local cell_index = (r - 1) * SoarNavGlobals.grid_cols + c
                table.insert(SoarNavGlobals.valid_cell_indices, cell_index)
            end
            
            cells_processed = cells_processed + 1
            SoarNavGlobals.grid_init_col = SoarNavGlobals.grid_init_col + 1
            if SoarNavGlobals.grid_init_col > SoarNavGlobals.grid_cols then
                SoarNavGlobals.grid_init_col = 1
                SoarNavGlobals.grid_init_row = SoarNavGlobals.grid_init_row + 1
            end
        end

        if SoarNavGlobals.grid_init_row > SoarNavGlobals.grid_rows then
            SoarNavGlobals.is_initializing = false
            SoarNavGlobals.grid_initialized = true
            SoarNavGlobals.grid_init_step = 0
            SoarNavGlobals.unvisited_cell_indices = {}
            for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                table.insert(SoarNavGlobals.unvisited_cell_indices, v)
            end
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Grid validation done: %d valid cells.", #SoarNavGlobals.valid_cell_indices))
            log_gcs(MAV_SEVERITY.NOTICE, 1, "Exploration grid ready.")
        end
    end
end

-- Marks the current grid cell as visited
local function update_visited_cell()
    local loc = ahrs:get_location()
    if not loc then return end
    local lat, lon = loc:lat()/1e7, loc:lng()/1e7
    local current_cell_index = get_cell_index_from_location(lat, lon)
    if current_cell_index and current_cell_index > 0 and current_cell_index <= #SoarNavGlobals.grid_cells then
        if current_cell_index ~= SoarNavGlobals.last_cell_index then
            if SoarNavGlobals.grid_cells[current_cell_index] then
                if SoarNavGlobals.grid_cells[current_cell_index].visit_count == 0 then
                    for i = #SoarNavGlobals.unvisited_cell_indices, 1, -1 do
                        if SoarNavGlobals.unvisited_cell_indices[i] == current_cell_index then
                            table.remove(SoarNavGlobals.unvisited_cell_indices, i)
                            break
                        end
                    end
                end
                SoarNavGlobals.grid_cells[current_cell_index].visit_count = SoarNavGlobals.grid_cells[current_cell_index].visit_count + 1
                SoarNavGlobals.last_cell_index = current_cell_index
            end
        end
    end
end

--[[ THERMAL MEMORY SYSTEM ]]--

-- Manages the thermal memory, removing expired hotspots
local function clean_and_get_hotspots()
    local valid_hotspots = {}
    local now_num = tonumber(tostring(millis()))
    local lifetime_ms = (safe_get(p_tmem_life, 1200)) * 1000
    for i = #SoarNavGlobals.thermal_hotspots, 1, -1 do
        local hotspot = SoarNavGlobals.thermal_hotspots[i]
        if hotspot and hotspot.timestamp then
            local hotspot_time_num = tonumber(tostring(hotspot.timestamp))
            if hotspot_time_num then
                local age_ms = now_num - hotspot_time_num
                if age_ms < lifetime_ms then
                    table.insert(valid_hotspots, 1, hotspot)
                end
            end
        end
    end
    SoarNavGlobals.thermal_hotspots = valid_hotspots
    return SoarNavGlobals.thermal_hotspots
end

-- Analyzes and records a thermal after exiting THERMAL mode
local function stop_and_record_thermal()
    SoarNavGlobals.is_monitoring_thermal = false
    if not SoarNavGlobals.current_thermal_stats or not SoarNavGlobals.current_thermal_stats.entry_location or SoarNavGlobals.current_thermal_stats.sample_count == 0 then
        log_gcs(MAV_SEVERITY.WARNING, 1, "Thermal exit detected, but no valid data sampled.")
        return
    end
    local avg_strength = SoarNavGlobals.current_thermal_stats.total_strength / SoarNavGlobals.current_thermal_stats.sample_count
    local max_strength = SoarNavGlobals.current_thermal_stats.max_strength
    local entry_loc = SoarNavGlobals.current_thermal_stats.entry_location
    local hotspot_lat, hotspot_lon = entry_loc:lat()/1e7, entry_loc:lng()/1e7
    local variance = calculate_thermal_variance(SoarNavGlobals.current_thermal_stats.samples)
    local consistency = variance < SoarNavConstants.THERMAL_CONSISTENCY_VARIANCE_THRESHOLD and "consistent" or "variable"
    local wind_vec = get_wind_vector()
    if wind_vec and wind_vec:x() and wind_vec:y() then
        local wind_x = wind_vec:x()
        local wind_y = wind_vec:y()
        local wind_vel = math.sqrt(wind_x^2 + wind_y^2)
        if wind_vel > 1.0 then
            local wind_heading_rad = atan2_safe(wind_y, wind_x)
            local upwind_bearing = math.deg(wind_heading_rad)
            local base_comp_time = safe_get(p_wind_comp, 60)
            local adaptive_comp_time = base_comp_time * math.max(0.5, math.min(2.0, avg_strength / 1.5))
            local offset_dist = wind_vel * adaptive_comp_time
            local corrected_loc = location_offset(hotspot_lat, hotspot_lon, upwind_bearing, offset_dist)
            if corrected_loc then
                hotspot_lat, hotspot_lon = corrected_loc:lat()/1e7, corrected_loc:lng()/1e7
            end
        end
    end

    local is_inside_area = false
    if SoarNavGlobals.use_polygon_area then
        if is_point_in_polygon(hotspot_lat, hotspot_lon, SoarNavGlobals.polygon_points) then
            is_inside_area = true
        end
    else
        local active_center = get_active_center_location()
        if active_center then
            local center_lat, center_lon = active_center:lat()/1e7, active_center:lng()/1e7
            local max_dist = safe_get(p_max_dist, 500)
            if haversine(center_lat, center_lon, hotspot_lat, hotspot_lon) <= max_dist then
                is_inside_area = true
            end
        end
    end

    if not is_inside_area then
        log_gcs(MAV_SEVERITY.INFO, 1, "Thermal ignored: out of area.")
        SoarNavGlobals.current_thermal_stats = {}
        return
    end

    local timestamp_saved = millis()
    local new_hotspot = {
        lat = hotspot_lat,
        lon = hotspot_lon,
        timestamp = timestamp_saved,
        avg_strength = avg_strength,
        max_strength = max_strength,
        consistency = consistency,
        duration = (timestamp_saved - SoarNavGlobals.current_thermal_stats.start_time) / 1000,
        wind_vec = wind_vec
    }
    table.insert(SoarNavGlobals.thermal_hotspots, new_hotspot)
    SoarNavGlobals.last_used_hotspot_timestamp = timestamp_saved
    if #SoarNavGlobals.thermal_hotspots > SoarNavConstants.max_hotspots then
        table.sort(SoarNavGlobals.thermal_hotspots, function(a, b)
            return (a.avg_strength or 0) < (b.avg_strength or 0)
        end)
        table.remove(SoarNavGlobals.thermal_hotspots, 1)
        log_gcs(MAV_SEVERITY.INFO, 2, "Weakest thermal removed from memory.")
    end
    log_thermal_event(new_hotspot)
    SoarNavGlobals.current_thermal_stats = {}
end

-- Samples the climb rate while in THERMAL mode
local function sample_thermal_strength()
    local ned_velocity = ahrs:get_velocity_NED()
    if ned_velocity then
        local vel_d = ned_velocity:z()
        local climb_rate = -vel_d
        SoarNavGlobals.current_thermal_stats.total_strength = SoarNavGlobals.current_thermal_stats.total_strength + climb_rate
        if climb_rate > SoarNavGlobals.current_thermal_stats.max_strength then
            SoarNavGlobals.current_thermal_stats.max_strength = climb_rate
        end
        table.insert(SoarNavGlobals.current_thermal_stats.samples, climb_rate)
        if #SoarNavGlobals.current_thermal_stats.samples > 10 then
            table.remove(SoarNavGlobals.current_thermal_stats.samples, 1)
        end
        SoarNavGlobals.current_thermal_stats.sample_count = SoarNavGlobals.current_thermal_stats.sample_count + 1
        SoarNavGlobals.last_thermal_sample_ms = millis()
    end
end

-- Initializes thermal monitoring upon entering THERMAL mode
local function start_thermal_monitoring()
    local loc = ahrs:get_location()
    if not loc then
        log_gcs(MAV_SEVERITY.WARNING, 1, "Cannot start thermal monitoring, location unavailable.")
        return
    end
    SoarNavGlobals.is_monitoring_thermal = true
    SoarNavGlobals.last_thermal_sample_ms = millis()
    SoarNavGlobals.current_thermal_stats = {
        entry_location = loc,
        max_strength = -99,
        total_strength = 0,
        sample_count = 0,
        samples = {},
        start_time = millis()
    }
    table.insert(SoarNavGlobals.thermal_entry_timestamps, millis())
end

--[[ AREA AND WAYPOINT LOGIC ]]--

-- Loads the custom flight area from a file on the SD card
local function read_polygon_file(filename)
    local f = io.open(filename, "r")
    if not f then
        log_gcs(MAV_SEVERITY.ERROR, 0, string.format("Could not open polygon file %s.", filename))
        return nil
    end
    SoarNavGlobals.polygon_points = {}
    SoarNavGlobals.polygon_bounds = {min_lat = 91, max_lat = -91, min_lon = 181, max_lon = -181}
    for line in f:lines() do
        if not line:match("^#") and line:match("%S") then
            local lat_str, lon_str = line:match("^%s*([%d%.%-]+)%s+([%d%.%-]+)%s*$")
            if lat_str and lon_str then
                local lat, lon = tonumber(lat_str), tonumber(lon_str)
                if lat and lon then
                    table.insert(SoarNavGlobals.polygon_points, {lat = lat, lon = lon})
                    SoarNavGlobals.polygon_bounds.min_lat = math.min(SoarNavGlobals.polygon_bounds.min_lat, lat)
                    SoarNavGlobals.polygon_bounds.max_lat = math.max(SoarNavGlobals.polygon_bounds.max_lat, lat)
                    SoarNavGlobals.polygon_bounds.min_lon = math.min(SoarNavGlobals.polygon_bounds.min_lon, lon)
                    SoarNavGlobals.polygon_bounds.max_lon = math.max(SoarNavGlobals.polygon_bounds.max_lon, lon)
                end
            end
        end
    end
    f:close()
    if #SoarNavGlobals.polygon_points < 3 then
        log_gcs(MAV_SEVERITY.ERROR, 0, string.format("Polygon file %s has less than 3 valid points. Reverting to radial.", filename))
        SoarNavGlobals.polygon_bounds = nil
        return nil
    end

    if #SoarNavGlobals.polygon_points >= 3 then
        local first_pt = SoarNavGlobals.polygon_points[1]
        local last_pt = SoarNavGlobals.polygon_points[#SoarNavGlobals.polygon_points]
        if first_pt.lat ~= last_pt.lat or first_pt.lon ~= last_pt.lon then
            table.insert(SoarNavGlobals.polygon_points, first_pt)
            log_gcs(MAV_SEVERITY.INFO, 2, "Auto-closed polygon.")
        end
    end

    log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Polygon loaded: %d points", #SoarNavGlobals.polygon_points))
    return true
end

-- Finds the RC channel assigned to the SOAR switch
local function find_soaring_rc_channel()
    for i = 1, 16 do
        if param:get(string.format("RC%d_OPTION", i)) == SoarNavConstants.rc_opt_soaring_active then
            SoarNavGlobals.detected_soaring_rc_channel = i
            log_gcs(MAV_SEVERITY.INFO, 1, string.format("Soaring RC detected on channel %d.", i))
            return true
        end
    end
    return false
end

-- Generates a random waypoint, ensuring it's inside the defined area
local function generate_target_around_point(center_lat, center_lon, radius_m)
    local target_loc = location_offset(center_lat, center_lon, math.random() * 360, math.sqrt(math.random()) * radius_m)
    if not target_loc then return nil, nil end
    local candidate_lat, candidate_lon = target_loc:lat()/1e7, target_loc:lng()/1e7
    local active_center = get_active_center_location()
    if not active_center then return nil, nil end
    if SoarNavGlobals.use_polygon_area then
        if not is_point_in_polygon(candidate_lat, candidate_lon, SoarNavGlobals.polygon_points) then
            return nil, nil
        end
    else
        local max_dist = safe_get(p_max_dist, 500)
        if max_dist > 0 then
            local center_point_lat, center_point_lon = active_center:lat()/1e7, active_center:lng()/1e7
            if haversine(center_point_lat, center_point_lon, candidate_lat, candidate_lon) > max_dist then
                return nil, nil
            end
        end
    end
    return candidate_lat, candidate_lon
end

local function log_new_waypoint(dist_to_wp)
    log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("New WP: %s | Dist: %.0fm", SoarNavGlobals.g_waypoint_source_info, dist_to_wp))
    log_gcs(MAV_SEVERITY.NOTICE, 2, string.format(" > Target: %.5f, %.5f", SoarNavGlobals.target_lat, SoarNavGlobals.target_lon))
end

-- Implements the low-energy strategy: target the best known thermal
local function select_best_thermal_waypoint()
    local valid_hotspots = clean_and_get_hotspots()
    if #valid_hotspots == 0 then return false end
    table.sort(valid_hotspots, function(a, b) return a.avg_strength > b.avg_strength end)
    local best_hotspot = valid_hotspots[1]
    local target_lat, target_lon = generate_target_around_point(best_hotspot.lat, best_hotspot.lon, 250)
    if target_lat and target_lon then
        SoarNavGlobals.target_lat, SoarNavGlobals.target_lon = target_lat, target_lon
        SoarNavGlobals.g_waypoint_source_info = string.format("Best Thermal (%.1fm/s)", best_hotspot.avg_strength)
        return true
    end
    return false
end

local update
local update_body

-- Main decision-making function for selecting the next target
local function search_for_new_waypoint()
    local active_center = get_active_center_location()
    if not active_center then return end
    
    local energy_status = assess_energy_state()
    
    if energy_status == "LOW" or energy_status == "CRITICAL" then
        log_gcs(MAV_SEVERITY.INFO, 2, string.format("Energy State: %s. Evaluating options...", energy_status))
        local valid_hotspots = clean_and_get_hotspots()
        if #valid_hotspots > 0 and select_best_thermal_waypoint() then
            log_gcs(MAV_SEVERITY.INFO, 2, "Select best thermal waypoint SUCCESS.")
            SoarNavGlobals.waypoint_start_time_ms = millis()
            SoarNavGlobals.waypoint_search_in_progress = false
            SoarNavGlobals.last_commanded_roll_deg = 0
            SoarNavGlobals.last_heading_error = 0
            local loc = ahrs:get_location()
            if loc then
                local dist_to_wp = haversine(loc:lat()/1e7, loc:lng()/1e7, SoarNavGlobals.target_lat, SoarNavGlobals.target_lon)
                log_new_waypoint(dist_to_wp)
            end
            return
        else
            if #valid_hotspots > 0 then
                log_gcs(MAV_SEVERITY.WARNING, 1, "Thermal selection failed, emergency search.")
            else
                log_gcs(MAV_SEVERITY.INFO, 2, "No thermals, emergency exploration.")
            end
            SoarNavGlobals.g_waypoint_source_info = string.format("Grid (%s Alt)", energy_status)
        end
    else
        log_gcs(MAV_SEVERITY.INFO, 2, "Altitude OK. Engaging standard search.")
    end

    local now = millis()
    local now_num = tonumber(tostring(now))
    local recent_success_count = 0
    local i = #SoarNavGlobals.thermal_entry_timestamps
    while i > 0 do
        local entry_time_num = tonumber(tostring(SoarNavGlobals.thermal_entry_timestamps[i]))
        if entry_time_num and (now_num - entry_time_num) > SoarNavConstants.thermal_history_window_ms then
            table.remove(SoarNavGlobals.thermal_entry_timestamps, i)
        else
            recent_success_count = recent_success_count + 1
        end
        i = i - 1
    end
    local MIN_TMEM_CHANCE = 15
    local MAX_TMEM_CHANCE = 85
    local MAX_SUCCESS_FOR_SCALING = 3
    local success_factor = math.min(recent_success_count / MAX_SUCCESS_FOR_SCALING, 1.0)
    local dynamic_tmem_chance = MIN_TMEM_CHANCE + (MAX_TMEM_CHANCE - MIN_TMEM_CHANCE) * success_factor
    local use_hotspot_logic = false
    local valid_hotspots_raw = clean_and_get_hotspots()
    local valid_hotspots = {}
    if (safe_get(p_tmem_enable, 1)) == 1 and #valid_hotspots_raw > 0 then
        if SoarNavGlobals.last_used_hotspot_timestamp ~= 0 then
            local last_ts_num = tonumber(tostring(SoarNavGlobals.last_used_hotspot_timestamp))
            for _, hotspot in ipairs(valid_hotspots_raw) do
                if hotspot.timestamp and tonumber(tostring(hotspot.timestamp)) ~= last_ts_num then
                    table.insert(valid_hotspots, hotspot)
                end
            end
        else
            valid_hotspots = valid_hotspots_raw
        end
        if #valid_hotspots > 0 and math.random(1, 100) <= dynamic_tmem_chance and not SoarNavGlobals.force_grid_after_reset then 
            use_hotspot_logic = true
        elseif #valid_hotspots == 0 and #valid_hotspots_raw > 0 and not SoarNavGlobals.logged_ignore_message then
            log_gcs(MAV_SEVERITY.INFO, 1, "Ignoring last thermal, using grid.")
            SoarNavGlobals.logged_ignore_message = true
        end
    end
    local source_for_log = "Unknown"
    local new_wp_found = false
    if use_hotspot_logic and not SoarNavGlobals.force_grid_after_reset then 
        local selected_hotspot
        if #valid_hotspots == 1 then
            selected_hotspot = valid_hotspots[1]
        else
            local idx1 = math.random(1, #valid_hotspots)
            local idx2 = math.random(1, #valid_hotspots)
            local h1 = valid_hotspots[idx1]
            local h2 = valid_hotspots[idx2]
            if h1.avg_strength and h2.avg_strength then
                selected_hotspot = (h1.avg_strength > h2.avg_strength) and h1 or h2
            else
                selected_hotspot = h1
            end
        end
        if selected_hotspot then
            local center_lat = selected_hotspot.lat
            local center_lon = selected_hotspot.lon
            local age_s = nil
            local drift_dist = nil
            if selected_hotspot.timestamp then
                local hotspot_time_num = tonumber(tostring(selected_hotspot.timestamp))
                if hotspot_time_num then
                    age_s = (now_num - hotspot_time_num) / 1000
                    local wind_vec = selected_hotspot.wind_vec or get_wind_vector()
                    if wind_vec and wind_vec:x() and wind_vec:y() then
                        local wind_vel = wind_vec:length()
                        if wind_vel > 1.0 and age_s > 10 then
                            local wind_heading_rad = atan2_safe(wind_vec:y(), wind_vec:x())
                            local wind_dir_bearing = (math.deg(wind_heading_rad) + 180 + 360) % 360
                            local drift_factor = math.min(SoarNavConstants.MAX_DRIFT_FACTOR, age_s / SoarNavConstants.MAX_DRIFT_AGE_S)
                            drift_dist = wind_vel * age_s * drift_factor
                            local drifted_loc = location_offset(center_lat, center_lon, wind_dir_bearing, drift_dist)
                            if drifted_loc then
                                center_lat = drifted_loc:lat() / 1e7
                                center_lon = drifted_loc:lng() / 1e7
                            end
                        end
                    end
                end
            end
            local candidate_lat, candidate_lon = generate_target_around_point(center_lat, center_lon, 250)
            if candidate_lat and candidate_lon then
                log_gcs(MAV_SEVERITY.INFO, 2, string.format("Targeting hotspot: avg %.1fm/s", selected_hotspot.avg_strength or 0))
                if drift_dist then
                    log_gcs(MAV_SEVERITY.INFO, 2, string.format("Drift prediction. Age: %.0fs, Dist: %.0fm", age_s, drift_dist))
                end
                SoarNavGlobals.target_lat, SoarNavGlobals.target_lon = candidate_lat, candidate_lon
                new_wp_found = true
                source_for_log = string.format("Drifting Thermal (Avg: %.1fm/s)", selected_hotspot.avg_strength or 0)
            end
        end
    end
    if not new_wp_found then
        if not SoarNavGlobals.grid_initialized or not SoarNavGlobals.valid_cell_indices or #SoarNavGlobals.valid_cell_indices == 0 then
            log_gcs(MAV_SEVERITY.WARNING, 1, "Grid not ready or no valid cells, using random fallback.")
            source_for_log = "Random Fallback"
            local center_point_lat, center_point_lon = active_center:lat()/1e7, active_center:lng()/1e7
            local max_dist = safe_get(p_max_dist, 500)
            local rand_dist = math.sqrt(math.random()) * max_dist
            local rand_bearing = math.random() * 360
            local fallback_loc = location_offset(center_point_lat, center_point_lon, rand_bearing, rand_dist)
            if fallback_loc then
                SoarNavGlobals.target_lat, SoarNavGlobals.target_lon = fallback_loc:lat()/1e7, fallback_loc:lng()/1e7
                new_wp_found = true
            end
        else
            if #SoarNavGlobals.unvisited_cell_indices == 0 then
                log_gcs(MAV_SEVERITY.NOTICE, 1, "Exploration complete, resetting.")
                for _, cell_idx in ipairs(SoarNavGlobals.valid_cell_indices) do
                    if cell_idx and cell_idx > 0 and cell_idx <= #SoarNavGlobals.grid_cells and SoarNavGlobals.grid_cells[cell_idx] then
                        SoarNavGlobals.grid_cells[cell_idx].visit_count = 0
                    end
                end
                SoarNavGlobals.unvisited_cell_indices = {}
                for _, v in ipairs(SoarNavGlobals.valid_cell_indices) do
                    table.insert(SoarNavGlobals.unvisited_cell_indices, v)
                end
                SoarNavGlobals.last_cell_index = nil
                SoarNavGlobals.force_grid_after_reset = true
            end

            if #SoarNavGlobals.unvisited_cell_indices > 0 then
                SoarNavGlobals.force_grid_after_reset = false
                local random_list_idx = math.random(1, #SoarNavGlobals.unvisited_cell_indices)
                local chosen_cell_index = SoarNavGlobals.unvisited_cell_indices[random_list_idx]
                table.remove(SoarNavGlobals.unvisited_cell_indices, random_list_idx)

                local row = math.floor((chosen_cell_index - 1) / SoarNavGlobals.grid_cols) + 1
                local col = ((chosen_cell_index - 1) % SoarNavGlobals.grid_cols) + 1
                local cell_height_lat = (SoarNavGlobals.grid_bounds.max_lat - SoarNavGlobals.grid_bounds.min_lat) / SoarNavGlobals.grid_rows
                local cell_width_lon = (SoarNavGlobals.grid_bounds.max_lon - SoarNavGlobals.grid_bounds.min_lon) / SoarNavGlobals.grid_cols
                local candidate_lat = SoarNavGlobals.grid_bounds.min_lat + (row - 1) * cell_height_lat + math.random() * cell_height_lat
                local candidate_lon = SoarNavGlobals.grid_bounds.min_lon + (col - 1) * cell_width_lon + math.random() * cell_width_lon
                SoarNavGlobals.target_lat, SoarNavGlobals.target_lon = candidate_lat, candidate_lon
                new_wp_found = true
                source_for_log = string.format("Cell: %d", chosen_cell_index)
            else
                log_gcs(MAV_SEVERITY.INFO, 2, "No unexplored valid cells found. Waiting for new conditions.")
                return
            end
        end
    end
    if new_wp_found then
        SoarNavGlobals.waypoint_start_time_ms = millis()
        SoarNavGlobals.g_waypoint_source_info = source_for_log
        SoarNavGlobals.waypoint_search_in_progress = false
        SoarNavGlobals.last_commanded_roll_deg = 0
        SoarNavGlobals.last_heading_error = 0
        SoarNavGlobals.last_progress_check_ms = 0
        SoarNavGlobals.distance_at_last_check = -1
        SoarNavGlobals.stuck_counter = 0
        local loc = ahrs:get_location()
        if loc then
            local dist_to_wp = haversine(loc:lat()/1e7, loc:lng()/1e7, SoarNavGlobals.target_lat, SoarNavGlobals.target_lon)
            log_new_waypoint(dist_to_wp)
        end
    end
end

-- Calculates an adaptive waypoint timeout based on wind
local function get_wp_timeout()
    local base_timeout = SoarNavConstants.wp_timeout_ms
    local wind_vec = get_wind_vector()
    if wind_vec and wind_vec:x() and wind_vec:y() then
        local wind_speed = wind_vec:length()
        if wind_speed and wind_speed > 1.0 then
            local adaptive_timeout = base_timeout * (1 + (wind_speed / 15))
            return math.min(adaptive_timeout, base_timeout * 2.5)
        end
    end
    return base_timeout
end

--[[ MAIN UPDATE LOOP ]]--

-- The main function containing the script's primary logic
update_body = function()
    local current_time_ms = millis()
    local current_time_num = tonumber(tostring(current_time_ms))
    local current_snav_max_dist = safe_get(p_max_dist, 0)
    if current_snav_max_dist ~= SoarNavGlobals.last_snav_max_dist_value then
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("SNAV_MAX_DIST changed from %.0f to %.0f. Re-initializing grid.", SoarNavGlobals.last_snav_max_dist_value, current_snav_max_dist))
        SoarNavGlobals.target_lat = nil
        SoarNavGlobals.target_lon = nil
        SoarNavGlobals.is_initializing = false
        SoarNavGlobals.grid_initialized = false
        SoarNavGlobals.last_snav_max_dist_value = current_snav_max_dist
        if current_snav_max_dist > 0 then
            SoarNavGlobals.use_polygon_area = false
            SoarNavGlobals.polygon_points = {}
            SoarNavGlobals.polygon_bounds = nil
            log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Radius Mode: %.0fm", current_snav_max_dist))
            if SoarNavGlobals.last_snav_max_dist_value == 0 then
                SoarNavGlobals.dynamic_center_location = ahrs:get_location()
                if SoarNavGlobals.dynamic_center_location then
                     log_gcs(MAV_SEVERITY.NOTICE, 1, "Radius center set to current location.")
                end
            end
        else
            if read_polygon_file(SoarNavConstants.polygon_filename) then
                SoarNavGlobals.use_polygon_area = true
                log_gcs(MAV_SEVERITY.NOTICE, 1, string.format("Polygon Mode: %d points loaded.", #SoarNavGlobals.polygon_points))
                SoarNavGlobals.dynamic_center_location = nil
            else
                log_gcs(MAV_SEVERITY.ERROR, 0, "Failed to load polygon, reverting to radial mode.")
                p_max_dist:set(500)
                SoarNavGlobals.use_polygon_area = false
                SoarNavGlobals.grid_initialized = false
                log_gcs(MAV_SEVERITY.WARNING, 0, "Forcing SNAV_MAX_DIST to 500 due to polygon load failure.")
            end
        end
    end
    if (safe_get(p_enable, 0)) == 0 then
        if SoarNavGlobals.script_state ~= SCRIPT_STATE.IDLE then set_script_state(SCRIPT_STATE.IDLE, "Script disabled by user.") end
        return update, 1000
    end
    if not validate_params() then
        set_script_state(SCRIPT_STATE.ERROR, "Invalid SNAV parameters detected. Disabling.")
        return update, 5000
    end
    if SoarNavGlobals.script_state == SCRIPT_STATE.ERROR then
        return update, 5000
    end
    if not SoarNavGlobals.rc_limits_read and arming:is_armed() then
        SoarNavGlobals.rc1_min = param:get('RC1_MIN') or 1000
        SoarNavGlobals.rc1_max = param:get('RC1_MAX') or 2000
        SoarNavGlobals.rc1_trim = param:get('RC1_TRIM') or 1500
        SoarNavGlobals.rc2_min = param:get('RC2_MIN') or 1000
        SoarNavGlobals.rc2_max = param:get('RC2_MAX') or 2000
        SoarNavGlobals.rc2_trim = param:get('RC2_TRIM') or 1500
        SoarNavGlobals.rc4_min = param:get('RC4_MIN') or 1000
        SoarNavGlobals.rc4_max = param:get('RC4_MAX') or 2000
        SoarNavGlobals.rc4_trim = param:get('RC4_TRIM') or 1500
        SoarNavGlobals.rc_limits_read = true
        log_gcs(MAV_SEVERITY.INFO, 1, string.format("RC1/RC2/RC4 limits read."))
    end
    local loc = ahrs:get_location()
    if not loc then
        SoarNavGlobals.location_error_count = SoarNavGlobals.location_error_count + 1
        if SoarNavGlobals.location_error_count > SoarNavConstants.max_location_errors then
            set_script_state(SCRIPT_STATE.ERROR, "Persistent location error, disabling.")
        end
        return update, 200
    else
        SoarNavGlobals.location_error_count = safe_decrement(SoarNavGlobals.location_error_count)
    end
    local last_rc_search_num = tonumber(tostring(SoarNavGlobals.last_rc_search_ms))
    if not SoarNavGlobals.detected_soaring_rc_channel and (current_time_num - last_rc_search_num) > SoarNavConstants.rc_search_interval_ms then
        find_soaring_rc_channel()
        SoarNavGlobals.last_rc_search_ms = current_time_ms
    end
    if arming:is_armed() and not SoarNavGlobals.grid_initialized and not SoarNavGlobals.is_initializing then
        local current_snav_max_dist_check = safe_get(p_max_dist, 0)
        if current_snav_max_dist_check == 0 and not SoarNavGlobals.use_polygon_area then
            if read_polygon_file(SoarNavConstants.polygon_filename) then
                SoarNavGlobals.use_polygon_area = true
            end
        end
        if SoarNavGlobals.use_polygon_area or (current_snav_max_dist_check > 0) then
            initialize_grid()
        end
    end
    if SoarNavGlobals.is_initializing then
        manage_grid_initialization()
        return update, 200
    end
    local pitch_input = rc:get_pwm(2)
    local yaw_input = rc:get_pwm(4)
    local roll_input = rc:get_pwm(1)
    local dz1 = param:get('RC1_DZ') or SoarNavConstants.rc_deadzone
    local dz2 = param:get('RC2_DZ') or SoarNavConstants.rc_deadzone
    local dz4 = param:get('RC4_DZ') or SoarNavConstants.rc_deadzone
    if SoarNavGlobals.script_state == SCRIPT_STATE.PILOT_OVERRIDE then
        local pilot_is_holding_input =
            (pitch_input and math.abs(pitch_input - SoarNavGlobals.rc2_trim) > dz2) or
            (yaw_input and math.abs(yaw_input - SoarNavGlobals.rc4_trim) > dz4) or
            (roll_input and math.abs(roll_input - SoarNavGlobals.rc1_trim) > dz1)
        if pilot_is_holding_input then
            SoarNavGlobals.last_pilot_input_ms = current_time_ms
        end
        check_pitch_gesture()
        check_roll_gesture()
        if not SoarNavGlobals.manual_override_active then
            local last_pilot_input_num = tonumber(tostring(SoarNavGlobals.last_pilot_input_ms))
            if not pilot_is_holding_input and last_pilot_input_num ~= 0 and (current_time_num - last_pilot_input_num) > SoarNavConstants.pilot_resume_delay_ms then
                set_script_state(SCRIPT_STATE.IDLE, "Sticks centered. Resuming.")
            end
        end
        return update, 200
    else
        if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING and ((pitch_input and math.abs(pitch_input - SoarNavGlobals.rc2_trim) > dz2) or (yaw_input and math.abs(yaw_input - SoarNavGlobals.rc4_trim) > dz4)) then
            SoarNavGlobals.last_pilot_input_ms = current_time_ms
            set_script_state(SCRIPT_STATE.PILOT_OVERRIDE, "Pilot override detected.")
            SoarNavGlobals.pitch_gesture_state = "idle"
            SoarNavGlobals.pitch_gesture_count = 0
            SoarNavGlobals.pitch_gesture_triggered_this_override = false
            SoarNavGlobals.roll_gesture_state = "idle"
            SoarNavGlobals.roll_gesture_count = 0
            SoarNavGlobals.manual_override_active = false
            return update, 200
        end
    end
    local current_mode = vehicle:get_mode()
    local is_in_thermal_mode = (current_mode == SoarNavConstants.mode_thermal)
    if (safe_get(p_tmem_enable, 1)) == 1 then
        if is_in_thermal_mode and not SoarNavGlobals.was_in_thermal_mode then
            start_thermal_monitoring()
        elseif not is_in_thermal_mode and SoarNavGlobals.was_in_thermal_mode then
            stop_and_record_thermal()
        end
    end
    SoarNavGlobals.was_in_thermal_mode = is_in_thermal_mode
    if SoarNavGlobals.is_monitoring_thermal then
        local time_since_sample_ms = nil
        if SoarNavGlobals.last_thermal_sample_ms then
            local last_sample_num = tonumber(tostring(SoarNavGlobals.last_thermal_sample_ms))
            if last_sample_num then
                time_since_sample_ms = current_time_num - last_sample_num
            end
        end
        local strength = SoarNavGlobals.current_thermal_stats.max_strength or 1.0
        if strength <= 0 then strength = 1.0 end
        local sample_interval = math.max(1000, math.min(5000, 3000 / strength))
        if time_since_sample_ms and (time_since_sample_ms > sample_interval) then
            sample_thermal_strength()
        end
    end
    local rc_val = 0
    if SoarNavGlobals.detected_soaring_rc_channel then
        rc_val = rc:get_pwm(SoarNavGlobals.detected_soaring_rc_channel) or 0
    end
    local script_switch_high = rc_val >= SoarNavConstants.rc_thresh_high
    local home = ahrs:get_home()
    local dist_from_home_3d = ahrs:get_relative_position_NED_home()
    local can_navigate = false
    if arming:is_armed() and script_switch_high and (current_mode == SoarNavConstants.mode_fbwb or current_mode == SoarNavConstants.mode_cruise) and home and dist_from_home_3d and ahrs:get_yaw_rad() and SoarNavGlobals.grid_initialized then
        local min_alt = safe_get(p_soar_alt_min, 40)
        local current_alt = -dist_from_home_3d:z()
        if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING or SoarNavGlobals.script_state == SCRIPT_STATE.THERMAL_PAUSE then
            if (current_alt >= (min_alt - 5)) then can_navigate = true end
        else
            if (current_alt >= min_alt) then can_navigate = true end
        end
    end
    if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING then
        if is_in_thermal_mode then
            set_script_state(SCRIPT_STATE.THERMAL_PAUSE, "Thermal detected. Pausing and monitoring.")
        elseif not can_navigate then
            set_script_state(SCRIPT_STATE.IDLE, "Navigation conditions not met.")
        end
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.THERMAL_PAUSE then
        if not is_in_thermal_mode then
            set_script_state(SCRIPT_STATE.IDLE, "Exited thermal, resuming.")
        end
    elseif SoarNavGlobals.script_state == SCRIPT_STATE.IDLE then
        if can_navigate then
            set_script_state(SCRIPT_STATE.NAVIGATING, "Navigation conditions met, starting.")
        end
    end
    if SoarNavGlobals.script_state == SCRIPT_STATE.NAVIGATING then
        if not SoarNavGlobals.target_lat and not SoarNavGlobals.waypoint_search_in_progress then
            log_gcs(MAV_SEVERITY.INFO, 1, "Searching for a new waypoint...")
            SoarNavGlobals.waypoint_search_in_progress = true
            SoarNavGlobals.logged_ignore_message = false
        end
        if SoarNavGlobals.waypoint_search_in_progress then
            search_for_new_waypoint()
        end
        if SoarNavGlobals.target_lat then
            local waypoint_start_time_num = tonumber(tostring(SoarNavGlobals.waypoint_start_time_ms))
            local time_on_current_wp = current_time_num - waypoint_start_time_num
            if waypoint_start_time_num > 0 and time_on_current_wp > SoarNavConstants.STUCK_GRACE_PERIOD_MS and not SoarNavGlobals.is_repositioning then
                local last_check_num = tonumber(tostring(SoarNavGlobals.last_progress_check_ms))
                if last_check_num == 0 or (current_time_num - last_check_num) > SoarNavConstants.STUCK_PROGRESS_CHECK_INTERVAL_MS then
                    if SoarNavGlobals.distance_at_last_check ~= -1 then
                        local progress_made = SoarNavGlobals.distance_at_last_check - SoarNavGlobals.distance_to_wp
                        if progress_made < SoarNavConstants.STUCK_MIN_PROGRESS_M then
                            SoarNavGlobals.stuck_counter = SoarNavGlobals.stuck_counter + 1
                            log_gcs(MAV_SEVERITY.INFO, 2, string.format("Stuck counter: %d", SoarNavGlobals.stuck_counter))
                        else
                            SoarNavGlobals.stuck_counter = 0
                        end
                    end
                    SoarNavGlobals.last_progress_check_ms = current_time_ms
                    SoarNavGlobals.distance_at_last_check = SoarNavGlobals.distance_to_wp
                end
            end
            
            local wind_vec = get_wind_vector()
            local wind_speed = wind_vec and wind_vec:length() or 0
            local adaptive_stuck_limit = math.floor(math.max(2, math.min(5, wind_speed / 4)))

            if SoarNavGlobals.stuck_counter >= adaptive_stuck_limit and not SoarNavGlobals.is_repositioning then
                SoarNavGlobals.original_target_lat = SoarNavGlobals.target_lat
                SoarNavGlobals.original_target_lon = SoarNavGlobals.target_lon
                SoarNavGlobals.is_repositioning = true
                local current_loc = ahrs:get_location()
                if wind_vec and current_loc then
                    local repo_dist_m = math.max(150, math.min(700, wind_speed * 60))
                    log_gcs(MAV_SEVERITY.INFO, 1, string.format("Stuck detected. Repositioning upwind, adaptive distance: %.0fm", repo_dist_m))
                    local wind_heading_rad = atan2_safe(wind_vec:y(), wind_vec:x())
                    local upwind_bearing = (math.deg(wind_heading_rad) + 180 + 360) % 360
                    local repo_loc = location_offset(current_loc:lat()/1e7, current_loc:lng()/1e7, upwind_bearing, repo_dist_m)
                    if repo_loc then
                        SoarNavGlobals.target_lat = repo_loc:lat()/1e7
                        SoarNavGlobals.target_lon = repo_loc:lng()/1e7
                        SoarNavGlobals.waypoint_start_time_ms = millis()
                        SoarNavGlobals.stuck_counter = 0
                        SoarNavGlobals.last_progress_check_ms = 0
                        SoarNavGlobals.distance_at_last_check = -1
                        log_gcs(MAV_SEVERITY.INFO, 1, "New tactical WP set upwind.")
                    end
                else
                    SoarNavGlobals.is_repositioning = false
                    SoarNavGlobals.target_lat = nil
                end
            end
            if waypoint_start_time_num > 0 and time_on_current_wp > get_wp_timeout() then
                log_gcs(MAV_SEVERITY.WARNING, 1, "Waypoint timeout. Generating new WP.")
                SoarNavGlobals.target_lat = nil
                SoarNavGlobals.waypoint_search_in_progress = true
                SoarNavGlobals.is_repositioning = false
            end
            if SoarNavGlobals.distance_to_wp ~= -1 and SoarNavGlobals.distance_to_wp < (safe_get(p_wp_radius, 30)) then
                if SoarNavGlobals.is_repositioning then
                    log_gcs(MAV_SEVERITY.INFO, 1, "Repositioned. Re-engaging original target.")
                    SoarNavGlobals.target_lat = SoarNavGlobals.original_target_lat
                    SoarNavGlobals.target_lon = SoarNavGlobals.original_target_lon
                    SoarNavGlobals.is_repositioning = false
                    SoarNavGlobals.original_target_lat = nil
                    SoarNavGlobals.original_target_lon = nil
                    SoarNavGlobals.waypoint_start_time_ms = millis()
                    SoarNavGlobals.distance_to_wp = -1
                    SoarNavGlobals.stuck_counter = 0
                    SoarNavGlobals.last_progress_check_ms = 0
                    SoarNavGlobals.distance_at_last_check = -1
                else
                    log_gcs(MAV_SEVERITY.INFO, 1, "Waypoint reached.")
                    SoarNavGlobals.target_lat = nil
                    SoarNavGlobals.distance_to_wp = -1
                    SoarNavGlobals.waypoint_search_in_progress = true
                end
                return update, 200
            end
            local curr_lat, curr_lon = loc:lat()/1e7, loc:lng()/1e7
            local target_heading = calculate_bearing(curr_lat, curr_lon, SoarNavGlobals.target_lat, SoarNavGlobals.target_lon)
            local current_heading = math.deg(ahrs:get_yaw_rad())
            local heading_error = (target_heading - current_heading + 540) % 360 - 180
            local last_grid_update_num = tonumber(tostring(SoarNavGlobals.last_grid_update_ms))
            if last_grid_update_num == 0 or (current_time_num - last_grid_update_num) > SoarNavConstants.grid_update_interval_ms then
                update_visited_cell()
                SoarNavGlobals.distance_to_wp = haversine(curr_lat, curr_lon, SoarNavGlobals.target_lat, SoarNavGlobals.target_lon)
                SoarNavGlobals.last_grid_update_ms = current_time_ms
                if (safe_get(p_log_lvl, 1)) >= 2 then
                    local roll_limit_for_log = safe_get(p_roll_limit, 30)
                    local p_gain_for_log = safe_get(p_nav_p, 0.6)
                    local d_gain_for_log = safe_get(p_nav_d, 0.05)
                    local error_derivative_for_log = (heading_error - SoarNavGlobals.last_heading_error) * 5
                    local p_term_for_log = heading_error * p_gain_for_log
                    local d_term_for_log = error_derivative_for_log * d_gain_for_log
                    local clamped_p_term_for_log = math.max(-roll_limit_for_log, math.min(roll_limit_for_log, p_term_for_log))
                    local smoothed_p_cmd_for_log = (SoarNavConstants.roll_smoothing_factor * clamped_p_term_for_log) + ((1.0 - SoarNavConstants.roll_smoothing_factor) * SoarNavGlobals.last_commanded_roll_deg)
                    local desired_roll_for_log = smoothed_p_cmd_for_log + d_term_for_log
                    desired_roll_for_log = math.max(-roll_limit_for_log, math.min(roll_limit_for_log, desired_roll_for_log))
                    local target_line = string.format("Nav to: %s", SoarNavGlobals.g_waypoint_source_info or "Unknown")
                    local wp_state_line = string.format("WP: D:%.0fm, Hdg Err:%+.0f, Roll:%+.1f", SoarNavGlobals.distance_to_wp, heading_error, desired_roll_for_log)
                    local visited, total_cells = 0, #SoarNavGlobals.valid_cell_indices
                    if total_cells > 0 then
                        visited = total_cells - #SoarNavGlobals.unvisited_cell_indices
                    end
                    local grid_percent = total_cells > 0 and (100 * visited / total_cells) or 0
                    local grid_line = string.format("Grid: %d/%d expl. %.0f%% | Curr. Cell: %d", visited, total_cells, grid_percent, SoarNavGlobals.last_cell_index or 0)
                    local valid_hotspots_log = clean_and_get_hotspots()
                    local tmem_count = #valid_hotspots_log
                    local tmem_line
                    if tmem_count > 0 then
                        local best_hotspot = valid_hotspots_log[1]
                        for i = 2, tmem_count do
                            if valid_hotspots_log[i].avg_strength and best_hotspot.avg_strength and valid_hotspots_log[i].avg_strength > best_hotspot.avg_strength then
                                best_hotspot = valid_hotspots_log[i]
                            end
                        end
                        tmem_line = string.format("Thermal mem: %d active (Best: +%.1fm/s)", tmem_count, best_hotspot.avg_strength or 0)
                    else
                        tmem_line = "Thermal mem: 0 active"
                    end
                    log_gcs(MAV_SEVERITY.INFO, 2, target_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, wp_state_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, grid_line)
                    log_gcs(MAV_SEVERITY.INFO, 2, tmem_line)
                end
            end
            local max_roll_angle = math.max(10, math.min(50, safe_get(p_roll_limit, 30)))
            local p_gain = math.max(0, safe_get(p_nav_p, 0.6))
            local d_gain = math.max(0, safe_get(p_nav_d, 0.05))
            local error_derivative = (heading_error - SoarNavGlobals.last_heading_error) * 5
            SoarNavGlobals.last_heading_error = heading_error
            local p_term = heading_error * p_gain
            local d_term = error_derivative * d_gain
            local raw_pd_command = p_term + d_term
            local smoothed_command
            if math.abs(raw_pd_command) > math.abs(SoarNavGlobals.last_commanded_roll_deg) then
                smoothed_command = (SoarNavConstants.roll_smoothing_factor * raw_pd_command) + ((1.0 - SoarNavConstants.roll_smoothing_factor) * SoarNavGlobals.last_commanded_roll_deg)
            else
                smoothed_command = raw_pd_command
            end
            local desired_roll_deg = math.max(-max_roll_angle, math.min(max_roll_angle, smoothed_command))
            SoarNavGlobals.last_commanded_roll_deg = desired_roll_deg
            local system_max_roll = safe_get(p_sys_roll_limit, 45)
            if system_max_roll <= 0 then system_max_roll = 45 end
            local roll_normalized = desired_roll_deg / system_max_roll
            local roll_pwm_value
            if roll_normalized > 0 then
                roll_pwm_value = SoarNavGlobals.rc1_trim + roll_normalized * (SoarNavGlobals.rc1_max - SoarNavGlobals.rc1_trim)
            else
                roll_pwm_value = SoarNavGlobals.rc1_trim + roll_normalized * (SoarNavGlobals.rc1_trim - SoarNavGlobals.rc1_min)
            end
            if SoarNavGlobals.rc_roll_channel then
                SoarNavGlobals.rc_roll_channel:set_override(math.floor(roll_pwm_value))
            else
                set_script_state(SCRIPT_STATE.ERROR, "Roll channel object is nil, cannot set override.")
            end
        end
    end
    return update, 200
end

-- Wrapper for update_body that includes pcall for safe error handling
update = function()
    local results = {pcall(update_body)}
    local ok = table.remove(results, 1)

    if not ok then
        log_gcs(MAV_SEVERITY.CRITICAL, 0, "SoarNav CRITICAL ERROR: " .. tostring(results[1]))
        return update, 5000
    end
    
    return table.unpack(results)
end

-- Script entry point
log_gcs(MAV_SEVERITY.INFO, 1, "SoarNav Script Initialized.")
return update()