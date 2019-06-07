/* clang-format off */

#define GPS_ALTITUDE_MASK   (0xFFFFFF00)  /* GPS altitude mask */
#define GPS_ALTITUDE_SHIFT  (8)           /* GPS altitude shift */
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
    FLYING_STATE_LANDED = 0,       /* Landed state */
    FLYING_STATE_TAKINGOFF,        /* Taking off state */
    FLYING_STATE_HOVERING,         /* Hovering state */
    FLYING_STATE_FLYING,           /* Flying state */
    FLYING_STATE_LANDING,          /* Landing state */
    FLYING_STATE_EMERGENCY,        /* Emergency state */
};

enum piloting_mode
{
    PILOTING_MODE_MANUAL = 0,      /* Manual piloting by the user */
    PILOTING_MODE_RETURN_HOME,     /* Automatic return home in progress */
    PILOTING_MODE_FLIGHT_PLAN,     /* Automatic flight plan in progress */
    PILOTING_MODE_FOLLOW_ME,       /* Automatic "follow-me" in progress */
};

struct metadata_v1_recording
{
    uint32_t frame_timestamp_h;    /* Frame timestamp (µs, monotonic), high 32 bits */
    uint32_t frame_timestamp_l;    /* Frame timestamp (µs, monotonic), low 32 bits */
    int16_t  drone_yaw;            /* Drone yaw/psi (rad), Q4.12 */
    int16_t  drone_pitch;          /* Drone pitch/theta (rad), Q4.12 */
    int16_t  drone_roll;           /* Drone roll/phi (rad), Q4.12 */
    int16_t  camera_pan;           /* Camera pan (rad), Q4.12 */
    int16_t  camera_tilt;          /* Camera tilt (rad), Q4.12 */
    int16_t  frame_w;              /* Frame view quaternion W, Q4.12 */
    int16_t  frame_x;              /* Frame view quaternion X, Q4.12 */
    int16_t  frame_y;              /* Frame view quaternion Y, Q4.12 */
    int16_t  frame_z;              /* Frame view quaternion Z, Q4.12 */
    int16_t  exposure_time;        /* Frame exposure time (ms), Q8.8 */
    int16_t  gain;                 /* Frame ISO gain */
    int8_t   wifi_rssi;            /* Wifi RSSI (dBm) */
    uint8_t  battery_percentage;   /* Battery charge percentage */
    int32_t  gps_latitude;         /* GPS latitude (deg), Q12.20 */
    int32_t  gps_longitude;        /* GPS longitude (deg), Q12.20 */
    int32_t  gps_altitude_and_sv;  /* Bits 31..8 = GPS altitude (m) Q16.8, bits 7..0 = SV count */
    int32_t  altitude;             /* Altitude relative to take-off (m), Q16.16 */
    uint32_t distance_from_home;   /* Distance from home (m), Q16.16 */
    int16_t  x_speed;              /* X speed (m/s), Q8.8 */
    int16_t  y_speed;              /* Y speed (m/s), Q8.8 */
    int16_t  z_speed;              /* Z speed (m/s), Q8.8 */
    uint8_t  state;                /* Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /* Bit 7 = animation, bits 6..0 = pilotingMode */
};

struct metadata_v1_streaming_basic
{
    uint16_t specific;             /* Identifier = 0x5031 */
    uint16_t length;               /* Size in 32 bits words = 6 */
    int16_t  drone_yaw;            /* Drone yaw/psi (rad), Q4.12 */
    int16_t  drone_pitch;          /* Drone pitch/theta (rad), Q4.12 */
    int16_t  drone_roll;           /* Drone roll/phi (rad), Q4.12 */
    int16_t  camera_pan;           /* Camera pan (rad), Q4.12 */
    int16_t  camera_tilt;          /* Camera tilt (rad), Q4.12 */
    int16_t  frame_w;              /* Frame view quaternion W, Q4.12 */
    int16_t  frame_x;              /* Frame view quaternion X, Q4.12 */
    int16_t  frame_y;              /* Frame view quaternion Y, Q4.12 */
    int16_t  frame_z;              /* Frame view quaternion Z, Q4.12 */
    int16_t  exposure_time;        /* Frame exposure time (ms), Q8.8 */
    int16_t  gain;                 /* Frame ISO gain */
    int8_t   wifi_rssi;            /* Wifi RSSI (dBm) */
    uint8_t  battery_percentage;   /* Battery charge percentage */
};

struct metadata_v1_streaming_extended
{
    uint16_t specific;             /* Identifier = 0x5031 */
    uint16_t length;               /* Size in 32 bits words = 13 */
    int16_t  drone_yaw;            /* Drone yaw/psi (rad), Q4.12 */
    int16_t  drone_pitch;          /* Drone pitch/theta (rad), Q4.12 */
    int16_t  drone_roll;           /* Drone roll/phi (rad), Q4.12 */
    int16_t  camera_pan;           /* Camera pan (rad), Q4.12 */
    int16_t  camera_tilt;          /* Camera tilt (rad), Q4.12 */
    int16_t  frame_w;              /* Frame view quaternion W, Q4.12 */
    int16_t  frame_x;              /* Frame view quaternion X, Q4.12 */
    int16_t  frame_y;              /* Frame view quaternion Y, Q4.12 */
    int16_t  frame_z;              /* Frame view quaternion Z, Q4.12 */
    int16_t  exposure_time;        /* Frame exposure time (ms), Q8.8 */
    int16_t  gain;                 /* Frame ISO gain */
    int8_t   wifi_rssi;            /* Wifi RSSI (dBm) */
    uint8_t  battery_percentage;   /* Battery charge percentage */
    int32_t  gps_latitude;         /* GPS latitude (deg), Q12.20 */
    int32_t  gps_longitude;        /* GPS longitude (deg), Q12.20 */
    int32_t  gps_altitude_and_sv;  /* Bits 31..8 = GPS altitude (m) Q16.8, bits 7..0 = SV count */
    int32_t  altitude;             /* Altitude relative to take-off (m), Q16.16 */
    uint32_t distance_from_home;   /* Distance from home (m), Q16.16 */
    int16_t  x_speed;              /* X speed (m/s), Q8.8 */
    int16_t  y_speed;              /* Y speed (m/s), Q8.8 */
    int16_t  z_speed;              /* Z speed (m/s), Q8.8 */
    uint8_t  state;                /* Bit 7 = binning, bits 6..0 = flyingState */
    uint8_t  mode;                 /* Bit 7 = animation, bits 6..0 = pilotingMode */
};
