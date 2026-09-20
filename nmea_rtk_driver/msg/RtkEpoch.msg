std_msgs/Header header

uint8 FIX_NONE=0
uint8 FIX_SINGLE=1
uint8 FIX_DIFFERENTIAL=2
uint8 FIX_PPS=3
uint8 FIX_RTK_FIXED=4
uint8 FIX_RTK_FLOAT=5
uint8 FIX_ESTIMATED=6
uint8 FIX_MANUAL=7
uint8 FIX_SIMULATION=8
uint8 FIX_RTK_UNSPECIFIED=9

time publish_stamp
time position_arrival_stamp
time covariance_arrival_stamp
time velocity_arrival_stamp
time gsa_arrival_stamp

string utc_time
string utc_date_ddmmyy
float64 utc_seconds_of_day
bool utc_valid
time utc_stamp

string position_source
string position_talker
bool position_valid
bool coordinates_valid
uint8 fix_quality
string fix_mode
string navigation_status
uint16 satellites_used
float64 latitude_deg
float64 longitude_deg

bool msl_height_valid
float64 msl_height_m
bool geoid_separation_valid
float64 geoid_separation_m
bool ellipsoid_height_valid
float64 ellipsoid_height_m

bool differential_age_valid
float64 differential_age_sec
string differential_station_id
bool hdop_valid
float64 hdop

bool gsa_valid
uint8 fix_dimension
bool pdop_valid
float64 pdop
bool gsa_hdop_valid
float64 gsa_hdop
bool vdop_valid
float64 vdop

bool position_covariance_valid
string position_covariance_source
float64[9] position_covariance_enu

bool gst_rms_valid
float64 gst_rms_m
bool gst_error_ellipse_valid
float64 gst_semi_major_std_m
float64 gst_semi_minor_std_m
float64 gst_orientation_deg
bool gst_position_std_valid
float64 gst_latitude_std_m
float64 gst_longitude_std_m
float64 gst_altitude_std_m

bool velocity_present
bool velocity_valid
bool velocity_covariance_valid
string velocity_source
string velocity_talker
string velocity_mode
string velocity_navigation_status
float64 speed_mps
float64 track_true_deg
geometry_msgs/Vector3 velocity_enu_mps
float64[9] velocity_covariance_enu
