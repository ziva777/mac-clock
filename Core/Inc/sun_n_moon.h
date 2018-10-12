#ifndef _SUN_N_MOON_C_
#define _SUN_N_MOON_C_

#include <time.h>
#include <math.h>

#define snm_RAD_TO_DEG  (180.0 / M_PI)
#define snm_DEG_TO_RAD  (1.0 / snm_RAD_TO_DEG)

static double TIMEZONES[] = {
    -12.00,
    -11.00,
    -10.00,
    -09.3001,
    -09.00,
    -08.00,
    -07.00,
    -06.00,
    -05.00,
    -04.00,
    -03.3001,
    -03.00,
    -02.00,
    -01.00,
    +00.00,
    +01.00,
    +02.00,
    +03.00,
    +03.3001,
    +04.00,
    +04.3001,
    +05.00,
    +05.3001,
    +05.4501,
    +06.00,
    +06.3001,
    +07.00,
    +08.00,
    +08.4501,
    +09.00,
    +09.3001,
    +10.00,
    +10.3001,
    +11.00,
    +12.00,
    +12.4501,
    +13.00,
    +14.00,
};
static const uint8_t N_TIMEZONES = sizeof(TIMEZONES) / sizeof(TIMEZONES[0]);
static const uint8_t MSK_ZONE = 17;

/* The set of twilights to calculate (types of rise/set events).
 */
typedef enum {
    /* Event ID for calculation of rising and setting times for astronomical
     * TWILIGHT:: In this case, the calculated time will be the time when the
     * center of the object is at -18 degrees of geometrical elevation below
     * the astronomical horizon. At this time astronomical observations are
     * possible because the sky is dark enough.
     */
    TWILIGHT_ASTRONOMICAL,
    /* Event ID for calculation of rising and setting times for nautical
     * TWILIGHT:: In this case, the calculated time will be the time when the
     * center of the object is at -12 degrees of geometric elevation below the
     * astronomical horizon.
     */
    TWILIGHT_NAUTICAL,
    /* Event ID for calculation of rising and setting times for civil TWILIGHT::
     * In this case, the calculated time will be the time when the center of the
     * object is at -6 degrees of geometric elevation below the astronomical
     * horizon.
     */
    TWILIGHT_CIVIL,
    /* The standard value of 34' for the refraction at the local horizon.
     */
    HORIZON_34arcmin
} TWILIGHT;

/* Aux for calculation.
 */
typedef struct {
    double jd_ut;
    double t;
    double tt_minus_ut;
    double slongitude;
    double sanomaly;
} snm_CalculatorAux;

/* Calc typedef.
 */
typedef struct {
    int day, month, year;
    int hour, minute, second;

    double latitude;
    double longitude;
    double timezone;

    TWILIGHT twilight;

    /* Values for azimuth, elevation, rise, set, and transit for the Sun.
     * Angles in radians, rise ... as Julian days in UT.
     * Distance in AU. 
     * Moon age is the number of days since last new Moon,
     * in days, from 0 to 29.5*/

    double sun_az;
    double sun_el;
    double sun_dist;
    double sun_rise;
    double sun_set;
    double sun_elev;
    double sun_transit;
    double sun_transit_elev;

    double moon_az;
    double moon_el;
    double moon_dist;
    double moon_rise;
    double moon_set;
    double moon_elev;
    double moon_transit;
    double moon_transit_elev;

    double moon_age;

    /* Internal values */
    snm_CalculatorAux aux;
} snm_Calculator;


void snm_CalculatorCreate(snm_Calculator *calc);
void snm_CalculatorSetTime(snm_Calculator *calc, 
                           int day, 
                           int month, 
                           int year);
void snm_CalculatorSetDate(snm_Calculator *calc, 
                           int hour, 
                           int minute, 
                           int second);
void snm_CalculatorSetPoint(snm_Calculator *calc, 
                            double latitude, 
                            double longitude);
void snm_CalculatorSetTwilight(snm_Calculator *calc, 
                               TWILIGHT twilight);
void snm_CalculatorCalc(snm_Calculator *calc);

struct tm snm_CalculatorGetDateAsTm(double jd, 
                                    double tz);
// void snm_CalculatorGetDateAsString(snm_Calculator *calc);

#endif // _SUN_N_MOON_C_
