/* clang-format off */

#define ALTITUDE_MASK       (0xFFFFFF00)  /* Altitude mask */
#define ALTITUDE_SHIFT      (8)           /* Altitude shift */
#define GPS_SV_COUNT_MASK   (0x000000FF)  /* GPS SV count mask */
#define GPS_SV_COUNT_SHIFT  (0)           /* GPS SV count shift */
#define FLYING_STATE_MASK   (0x7F)        /* Flying state mask */
#define FLYING_STATE_SHIFT  (0)           /* Flying state shift */
#define BINNING_MASK        (0x80)        /* Binning mask */
#define BINNING_SHIFT       (7)           /* Binning shift */
#define PILOTING_MODE_MASK  (0x7F)        /* Piloting mode mask */
#define PILOTING_MODE_SHIFT (0)           /* Piloting mode shift */
#define ANIMATION_MASK      (0x80)        /* Animation mask */
#define ANIMATION_SHIFT     (7)           /* Animation shift */

enum flying_state
{
    FLYING_STATE_LANDED = 0,         /* Landed state */
    FLYING_STATE_TAKINGOFF,          /* Taking off state */
    FLYING_STATE_HOVERING,           /* Hovering state */
    FLYING_STATE_FLYING,             /* Flying state */
    FLYING_STATE_LANDING,            /* Landing state */
    FLYING_STATE_EMERGENCY,          /* Emergency state */
    FLYING_STATE_USER_TAKEOFF,       /* User take off state */
    FLYING_STATE_MOTOR_RAMPING,      /* Motor ramping state */
    FLYING_STATE_EMERGENCY_LANDING,  /* Emergency landing state */
};

enum piloting_mode
{
    PILOTING_MODE_MANUAL = 0,      /* Manual piloting by the user */
    PILOTING_MODE_RETURN_HOME,     /* Automatic return home in progress */
    PILOTING_MODE_FLIGHT_PLAN,     /* Automatic flight plan in progress */
    PILOTING_MODE_TRACKING,        /* Automatic tracking in progress */
    PILOTING_MODE_FOLLOW_ME = PILOTING_MODE_TRACKING,
    PILOTING_MODE_MAGIC_CARPET,    /* Automatic "magic carpet" test in progress */
    PILOTING_MODE_MOVE_TO,         /* Automatic "move to" in progress */
};

enum followme_anim
{
    FOLLOW_ME_ANIMATION_NONE = 0,  /* No animation in progress */
    FOLLOW_ME_ANIMATION_ORBIT,     /* Follow-me orbit animation in progress */
    FOLLOW_ME_ANIMATION_BOOMERANG, /* Follow-me boomerang animation in progress */
    FOLLOW_ME_ANIMATION_PARABOLA,  /* Follow-me parabola animation in progress */
    FOLLOW_ME_ANIMATION_ZENITH,    /* Follow-me zenith animation in progress */
};

enum automation_anim
{
    AUTOMATION_ANIMATION_NONE = 0,         /* No animation in progress */
    AUTOMATION_ANIMATION_ORBIT,            /* Orbit animation in progress */
    AUTOMATION_ANIMATION_BOOMERANG,        /* Boomerang animation in progress */
    AUTOMATION_ANIMATION_PARABOLA,         /* Parabola animation in progress */
    AUTOMATION_ANIMATION_DOLLY_SLIDE,      /* Dolly slide animation in progress */
    AUTOMATION_ANIMATION_DOLLY_ZOOM,       /* Dolly zoom animation in progress */
    AUTOMATION_ANIMATION_REVEAL_VERT,      /* Vertical reveal animation in progress */
    AUTOMATION_ANIMATION_REVEAL_HORZ,      /* Horizontal reveal animation in progress */
    AUTOMATION_ANIMATION_PANORAMA_HORZ,    /* Horizontal panorama animation in progress */
    AUTOMATION_ANIMATION_CANDLE,           /* Candle animation in progress */
    AUTOMATION_ANIMATION_FLIP_FRONT,       /* Front filp animation in progress */
    AUTOMATION_ANIMATION_FLIP_BACK,        /* Back flip animation in progress */
    AUTOMATION_ANIMATION_FLIP_LEFT,        /* Left flip animation in progress */
    AUTOMATION_ANIMATION_FLIP_RIGHT,       /* Right flip animation in progress */
    AUTOMATION_ANIMATION_TWISTUP,          /* Twist up animation in progress */
    AUTOMATION_ANIMATION_POSITION_TWISTUP, /* Postion twist up animation in progress */
};

enum thermal_calib_state {
	THERMAL_CALIB_STATE_DONE = 0,
	THERMAL_CALIB_STATE_REQUESTED,
	THERMAL_CALIB_STATE_IN_PROGRESS,
};

struct metadata_v2_base
{
    uint16_t id;                   /* Identifier = 0x5032 */
    uint16_t length;               /* Structure size in 32 bits words excluding the id and length
                                    * fields and including extensions */
    int32_t  ground_distance;      /* Best ground distance estimation (m), Q16.16 */
    int32_t  latitude;             /* Absolute latitude (deg), Q10.22 */
    int32_t  longitude;            /* Absolute longitude (deg), Q10.22 */
    int32_t  altitude_and_sv;      /* Bits 31..8 = altitude (m) Q16.8, bits 7..0 = GPS SV count */
    int16_t  north_speed;          /* North speed (m/s), Q8.8 */
    int16_t  east_speed;           /* East speed (m/s), Q8.8 */
    int16_t  down_speed;           /* Down speed (m/s), Q8.8 */
    int16_t  air_speed;            /* Speed relative to air (m/s), negative means no data, Q8.8 */
    int16_t  drone_w;              /* Drone quaternion W, Q2.14 */
    int16_t  drone_x;              /* Drone quaternion X, Q2.14 */
    int16_t  drone_y;              /* Drone quaternion Y, Q2.14 */
    int16_t  drone_z;              /* Drone quaternion Z, Q2.14 */
    int16_t  frame_w;              /* Frame view quaternion W, Q2.14 */
    int16_t  frame_x;              /* Frame view quaternion X, Q2.14 */
    int16_t  frame_y;              /* Frame view quaternion Y, Q2.14 */
    int16_t  frame_z;              /* Frame view quaternion Z, Q2.14 */
    int16_t  camera_pan;           /* Camera pan (rad), Q4.12 */
    int16_t  camera_tilt;          /* Camera tilt (rad), Q4.12 */
    uint16_t exposure_time;        /* Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /* Frame ISO gain */
    uint8_t  state;                /* Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /* Bit 7 = animation, bits 6..0 = pilotingMode */
    int8_t   wifi_rssi;            /* Wifi RSSI (dBm) */
    uint8_t  battery_percentage;   /* Battery charge percentage */
};

struct metadata_v3_base
{
    uint16_t id;                   /* Identifier = 0x5033 */
    uint16_t length;               /* Structure size in 32 bits words excluding the id and length
                                    * fields and including extensions */
    int32_t  ground_distance;      /* Best ground distance estimation (m), Q16.16 */
    int32_t  latitude;             /* Absolute latitude (deg), Q10.22 */
    int32_t  longitude;            /* Absolute longitude (deg), Q10.22 */
    int32_t  altitude_and_sv;      /* Bits 31..8 = altitude (m) Q16.8, bits 7..0 = GPS SV count */
    int16_t  north_speed;          /* North speed (m/s), Q8.8 */
    int16_t  east_speed;           /* East speed (m/s), Q8.8 */
    int16_t  down_speed;           /* Down speed (m/s), Q8.8 */
    int16_t  air_speed;            /* Speed relative to air (m/s), negative means no data, Q8.8 */
    int16_t  drone_w;              /* Drone quaternion W, Q2.14 */
    int16_t  drone_x;              /* Drone quaternion X, Q2.14 */
    int16_t  drone_y;              /* Drone quaternion Y, Q2.14 */
    int16_t  drone_z;              /* Drone quaternion Z, Q2.14 */
    int16_t  frame_base_w;         /* Frame base view quaternion W (without pan/tilt), Q2.14 */
    int16_t  frame_base_x;         /* Frame base view quaternion X (without pan/tilt), Q2.14 */
    int16_t  frame_base_y;         /* Frame base view quaternion Y (without pan/tilt), Q2.14 */
    int16_t  frame_base_z;         /* Frame base view quaternion Z (without pan/tilt), Q2.14 */
    int16_t  frame_w;              /* Frame view quaternion W, Q2.14 */
    int16_t  frame_x;              /* Frame view quaternion X, Q2.14 */
    int16_t  frame_y;              /* Frame view quaternion Y, Q2.14 */
    int16_t  frame_z;              /* Frame view quaternion Z, Q2.14 */
    uint16_t exposure_time;        /* Frame exposure time (ms), Q8.8 */
    uint16_t gain;                 /* Frame ISO gain */
    uint16_t awb_r_gain;           /* White balance R/G gain, Q2.14 */
    uint16_t awb_b_gain;           /* White balance B/G gain, Q2.14 */
    uint16_t picture_hfov;         /* Picture horizontal FOV (deg), Q8.8 */
    uint16_t picture_vfov;         /* Picture vertical FOV (deg), Q8.8 */
    uint32_t link_quality;         /* Bits 31..8 = link goodput (kbit/s),
                                    * bits 7..0 = link quality (0-5) */
    int8_t   wifi_rssi;            /* Wifi RSSI (dBm) */
    uint8_t  battery_percentage;   /* Battery charge percentage */
    uint8_t  state;                /* Flying state */
    uint8_t  mode;                 /* Bit 7 = animation, bits 6..0 = pilotingMode */
};

struct metadata_ext
{
    uint16_t ext_id;               /* Extension structure id */
    uint16_t ext_length;           /* Extension structure size in 32 bits words excluding the
                                    * ext_id and ext_size fields */
    [...]                          /* Extension fields */
};

struct metadata_timestamp_ext
{
    uint16_t ext_id;               /* Extension structure id = 0x4531 */
    uint16_t ext_length;           /* Extension structure size in 32 bits words excluding the
                                    * ext_id and ext_size fields */
    uint32_t frame_timestamp_h;    /* Frame timestamp (µs, monotonic), high 32 bits */
    uint32_t frame_timestamp_l;    /* Frame timestamp (µs, monotonic), low 32 bits */
};

struct metadata_followme_ext
{
    uint16_t ext_id;               /* Extension structure id = 0x4532 */
    uint16_t ext_length;           /* Extension structure size in 32 bits words excluding the
                                    * ext_id and ext_size fields */
    int32_t  target_latitude;      /* Target latitude (deg), Q10.22 */
    int32_t  target_longitude;     /* Target longitude (deg), Q10.22 */
    int32_t  target_altitude;      /* Target altitude ASL (m) Q16.16 */
    uint8_t  followme_mode;        /* Follow-me feature bit field
                                    *  - bit 0: follow-me enabled (0 = disabled, 1 = enabled)
                                    *  - bit 1: mode (0 = look-at-me, 1 = follow-me)
                                    *  - bit 2: angle mode (0 = unlocked, 1 = locked)
                                    *  - bit 3-7: reserved for future use */
    uint8_t  followme_animation;   /* Follow-me animation (0 means no animation in progress) */
    uint8_t  reserved1;            /* Reserved for future use */
    uint8_t  reserved2;            /* Reserved for future use */
    uint32_t reserved3;            /* Reserved for future use */
    uint32_t reserved4;            /* Reserved for future use */
};

struct metadata_automation_ext
{
    uint16_t ext_id;                   /* Extension structure id = 0x4533 */
    uint16_t ext_length;               /* Extension structure size in 32 bits words excluding the
                                        * ext_id and ext_size fields */
    int32_t  framing_target_latitude;  /* Framing target latitude (deg), Q10.22 */
    int32_t  framing_target_longitude; /* Framing target longitude (deg), Q10.22 */
    int32_t  framing_target_altitude;  /* Framing target altitude ASL (m) Q16.16 */
    int32_t  flight_destination_latitude;   /* Flight destination latitude (deg), Q10.22 */
    int32_t  flight_destination_longitude;  /* Flight destination longitude (deg), Q10.22 */
    int32_t  flight_destination_altitude;   /* Flight destination altitude ASL (m) Q16.16 */
    uint8_t  automation_animation;     /* Automation animation (0 means no animation in progress) */
    uint8_t  automation_flags;         /* Automation features bit field
                                        *  - bit 0: follow-me enabled (0 = disabled, 1 = enabled)
                                        *  - bit 1: look-at-me enabled (0 = disabled, 1 = enabled)
                                        *  - bit 2: angle locked (0 = unlocked, 1 = locked)
                                        *  - bit 3-7: reserved for future use */
    uint16_t reserved;                 /* Reserved for future use */
};

struct metadata_thermal_ext
{
    uint16_t ext_id;               /* Extension structure id = 0x4534 */
    uint16_t ext_length;           /* Extension structure size in 32 bits words excluding the
                                    * ext_id and ext_size fields */
    int16_t  min_x;                /* Minimum temperature spot x-coordinate (relative to frame width), Q11.5 */
    int16_t  min_y;                /* Minimum temperature spot y-coordinate (relative to frame height), Q11.5 */
    int16_t  min_temp;             /* Minimum temperature spot temperature value (K) Q11.5 */
    int16_t  max_x;                /* Maximum temperature spot x-coordinate (relative to frame width), Q11.5 */
    int16_t  max_y;                /* Maximum temperature spot y-coordinate (relative to frame height), Q11.5 */
    int16_t  max_temp;             /* Maximum temperature spot temperature value (K) Q11.5 */
    int16_t  probe_x;              /* Probe temperature x-coordinate (relative to frame width), Q11.5 */
    int16_t  probe_y;              /* Probe temperature y-coordinate (relative to frame height), Q11.5 */
    int16_t  probe_temp;           /* Probe temperature temperature value (K) Q11.5 */
    uint8_t  calib_state;          /* Calibration state */
    uint8_t  flags;                /* Validity flags
                                    *  - bit 0: minimum temperature spot (0 = invalid, 1 = valid)
                                    *  - bit 1: maximum temperature spot (0 = invalid, 1 = valid)
                                    *  - bit 2: probe temperature (0 = invalid, 1 = valid)
                                    *  - bit 3-7: reserved for future use */
};
