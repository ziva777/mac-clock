#include "sun_n_moon.h"

#include <stdio.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
#define STACK_OPTIMIZED

#ifdef STACK_OPTIMIZED
#   define STATIC_OPT static
#else
#   define STATIC_OPT 
#endif

#define FALSE   0
#define TRUE    1

/** Two times Pi. */
#define TWO_PI          (2.0 * M_PI)
/** Four times Pi. */
#define FOUR_PI         (4.0 * M_PI)
/** The inverse of two times Pi. */
#define TWO_PI_INVERSE  (1.0 / TWO_PI)

/** Radians to hours. */
#define RAD_TO_HOUR (180.0 / (15.0 * M_PI))
/** Radians to days. */
#define RAD_TO_DAY  (RAD_TO_HOUR / 24.0)
    
    
/** Astronomical Unit in km. As defined by JPL. */
const static double AU = 149597870.691;
/** Earth equatorial radius in km. IERS 2003 Conventions. */
const static double EARTH_RADIUS = 6378.1366;

/** Length of a sidereal day in days according to IERS Conventions. */
const static double SIDEREAL_DAY_LENGTH = 1.00273781191135448;
/** Julian century conversion constant = 100 * days per year. */
const static double JULIAN_DAYS_PER_CENTURY = 36525.0;
/** Seconds in one day. */
const static double SECONDS_PER_DAY = 86400.0;

/** Our default epoch.
 * The Julian Day which represents noon on 2000-01-01. */
const static double J2000 = 2451545.0;

//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
double JulianDay(int year, int month, int day, int h, int m, int s)
{
    // The conversion formulas are from Meeus, chapter 7.
    int julian = FALSE;

    if (year < 1582 ||
            (year == 1582 && month <= 10) ||
            (year == 1582 && month == 10 && day < 15))
        julian = TRUE;

    int D = day;
    int M = month;
    int Y = year;

    if (M < 3) {
        Y--;
        M += 12;
    }

    int A = Y / 100;
    int B = (julian ? 0 : 2 - A + A / 4);

    double dayFraction = (h + (m + (s / 60.0)) / 60.0) / 24.0;
    double jd = dayFraction +
            (int) (365.25 * (Y + 4716)) +
            (int) (30.6001 * (M + 1)) + D + B - 1524.5;
    return jd;
}

double TtMinusUt(int year, int month, int day) 
{
    double tt_minus_ut = 0.0;

    if (year > -600 && year < 2200) {
        double x = year + (month - 1 + day / 30.0) / 12.0;
        double x2 = x * x, x3 = x2 * x, x4 = x3 * x;
        if (year < 1600) {
            tt_minus_ut =
                    10535.328003326353 -
                    9.995238627481024 * x +
                    0.003067307630020489 * x2 -
                    7.76340698361363E-6 * x3 +
                    3.1331045394223196E-9 * x4 +
                    8.225530854405553E-12 * x2 * x3 -
                    7.486164715632051E-15 * x4 * x2 +
                    1.9362461549678834E-18 * x4 * x3 -
                    8.489224937827653E-23 * x4 * x4;
        } else {
            tt_minus_ut =
                    -1027175.3477559977 +
                    2523.256625418965 * x -
                    1.885686849058459 * x2 +
                    5.869246227888417E-5 * x3 +
                    3.3379295816475025E-7 * x4 +
                    1.7758961671447929E-10 * x2 * x3 -
                    2.7889902806153024E-13 * x2 * x4 +
                    1.0224295822336825E-16 * x3 * x4 -
                    1.2528102370680435E-20 * x4 * x4;
        }
    }

    return tt_minus_ut;
}

void SetUTDate(snm_Calculator *calc, double jd) 
{
    calc->aux.jd_ut = jd;
    calc->aux.t = (jd + calc->aux.tt_minus_ut / SECONDS_PER_DAY  - J2000) /
            JULIAN_DAYS_PER_CENTURY;
}

void SetTimes(snm_Calculator *calc)
{
    double jd = JulianDay(calc->year, 
                          calc->month,
                          calc->day,
                          calc->hour,
                          calc->minute,
                          calc->second);

    if (jd < 2299160.0 && jd >= 2299150.0) {
        fprintf(stderr, 
                "Invalid julian day %f. "
                "This date does not exist.\n", 
                jd);
        return;
    }

    calc->aux.tt_minus_ut = TtMinusUt(calc->year, 
                                      calc->month, 
                                      calc->day);
    SetUTDate(calc, jd);
}

double NormalizeRadians(double r) {
    if (r < 0.0 && r >= -TWO_PI)
        return r + TWO_PI;

    if (r >= TWO_PI && r < FOUR_PI)
        return r - TWO_PI;

    if (r >= 0.0 && r < TWO_PI)
        return r;

    r -= TWO_PI * floor(r * TWO_PI_INVERSE);

    if (r < 0.0)
        r += TWO_PI;

    return r;
}

void GetSun(snm_Calculator *calc, double *params) 
{
    STATIC_OPT double lon, anom, c, M1, M2, d, slatitude, ecc, v, sdistance;

    // SUN PARAMETERS (Formulae from "Calendrical Calculations")
    lon = (280.46645 +
                  36000.76983 * calc->aux.t +
                  .0003032 * calc->aux.t * calc->aux.t);
    anom = (357.5291 +
                   35999.0503 * calc->aux.t -
                   .0001559 * calc->aux.t * calc->aux.t -
                   4.8E-07 * calc->aux.t * calc->aux.t * calc->aux.t);
    calc->aux.sanomaly = anom * snm_DEG_TO_RAD;
    c = (1.9146 - .004817 * calc->aux.t - .000014 * calc->aux.t * calc->aux.t) * sin(calc->aux.sanomaly);

    c = c + (.019993 - .000101 * calc->aux.t) * sin(2 * calc->aux.sanomaly);
    // Correction to the mean ecliptic longitude
    c = c + .00029 * sin(3.0 * calc->aux.sanomaly);

    // Now, let calculate nutation and aberration
    M1 = (124.90 - 1934.134 * calc->aux.t + 0.002063 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    M2 = (201.11 + 72001.5377 * calc->aux.t + 0.00057 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    d = -.00569 - .0047785 * sin(M1) - .0003667 * sin(M2);

    // apparent longitude (error<0.003 deg)
    calc->aux.slongitude = lon + c + d;
    // Sun's ecliptic latitude is always negligible
    slatitude = 0;
    // Eccentricity
    ecc = .016708617 - 4.2037E-05 * calc->aux.t - 1.236E-07 * calc->aux.t * calc->aux.t;
    // True anomaly
    v = calc->aux.sanomaly + c * snm_DEG_TO_RAD;
    sdistance = 1.000001018 * (1.0 - ecc * ecc) /
            (1.0 + ecc * cos(v)); // In UA

    params[0] = calc->aux.slongitude;
    params[1] = slatitude;
    params[2] = sdistance;
    params[3] = atan(696000 / (AU * sdistance));
}

void GetMoon(snm_Calculator *calc, double *params) 
{
    // MOON PARAMETERS (Formulae from "Calendrical Calculations")
    STATIC_OPT double phase, anomaly, node, E, l, longitude, latitude;
    STATIC_OPT double M1, M2, d, Psin;
    STATIC_OPT double parallax, distance;

    phase = NormalizeRadians((297.8502042 + 445267.1115168 * calc->aux.t - 0.00163 * calc->aux.t * calc->aux.t + calc->aux.t * calc->aux.t * calc->aux.t / 538841 - calc->aux.t * calc->aux.t * calc->aux.t * calc->aux.t / 65194000) * snm_DEG_TO_RAD);

    // Anomalistic phase
    anomaly = (134.9634114 + 477198.8676313 * calc->aux.t + .008997 * calc->aux.t * calc->aux.t + calc->aux.t * calc->aux.t * calc->aux.t / 69699 - calc->aux.t * calc->aux.t * calc->aux.t * calc->aux.t / 14712000);
    anomaly = anomaly * snm_DEG_TO_RAD;

    // Degrees from ascending node
    node = (93.2720993 + 483202.0175273 * calc->aux.t - 0.0034029 * calc->aux.t * calc->aux.t - calc->aux.t * calc->aux.t * calc->aux.t / 3526000 + calc->aux.t * calc->aux.t * calc->aux.t * calc->aux.t / 863310000);
    node = node * snm_DEG_TO_RAD;

    E = 1.0 - (.002495 + 7.52E-06 * (calc->aux.t + 1.0)) * (calc->aux.t + 1.0);

    // Now longitude, with the three main correcting terms of evection,
    // variation, and equation of year, plus other terms (error<0.01 deg)
    // P. Duffet's MOON program taken as reference
    l = (218.31664563 + 481267.8811958 * calc->aux.t - .00146639 * calc->aux.t * calc->aux.t + calc->aux.t * calc->aux.t * calc->aux.t / 540135.03 - calc->aux.t * calc->aux.t * calc->aux.t * calc->aux.t / 65193770.4);
    l += 6.28875 * sin(anomaly) + 1.274018 * sin(2 * phase - anomaly) + .658309 * sin(2 * phase);
    l +=  0.213616 * sin(2 * anomaly) - E * .185596 * sin(calc->aux.sanomaly) - 0.114336 * sin(2 * node);
    l += .058793 * sin(2 * phase - 2 * anomaly) + .057212 * E * sin(2 * phase - anomaly - calc->aux.sanomaly) + .05332 * sin(2 * phase + anomaly);
    l += .045874 * E * sin(2 * phase - calc->aux.sanomaly) + .041024 * E * sin(anomaly - calc->aux.sanomaly) - .034718 * sin(phase) - E * .030465 * sin(calc->aux.sanomaly + anomaly);
    l += .015326 * sin(2 * (phase - node)) - .012528 * sin(2 * node + anomaly) - .01098 * sin(2 * node - anomaly) + .010674 * sin(4 * phase - anomaly);
    l += .010034 * sin(3 * anomaly) + .008548 * sin(4 * phase - 2 * anomaly);
    l += -E * .00791 * sin(calc->aux.sanomaly - anomaly + 2 * phase) - E * .006783 * sin(2 * phase + calc->aux.sanomaly) + .005162 * sin(anomaly - phase) + E * .005 * sin(calc->aux.sanomaly + phase);
    l += .003862 * sin(4 * phase) + E * .004049 * sin(anomaly - calc->aux.sanomaly + 2 * phase) + .003996 * sin(2 * (anomaly + phase)) + .003665 * sin(2 * phase - 3 * anomaly);
    l += E * 2.695E-3 * sin(2 * anomaly - calc->aux.sanomaly) + 2.602E-3 * sin(anomaly - 2*(node+phase));
    l += E * 2.396E-3 * sin(2*(phase - anomaly) - calc->aux.sanomaly) - 2.349E-3 * sin(anomaly+phase);
    l += E * E * 2.249E-3 * sin(2*(phase-calc->aux.sanomaly)) - E * 2.125E-3 * sin(2*anomaly+calc->aux.sanomaly);
    l += -E * E * 2.079E-3 * sin(2*calc->aux.sanomaly) + E * E * 2.059E-3 * sin(2*(phase-calc->aux.sanomaly)-anomaly);
    l += -1.773E-3 * sin(anomaly+2*(phase-node)) - 1.595E-3 * sin(2*(node+phase));
    l += E * 1.22E-3 * sin(4*phase-calc->aux.sanomaly-anomaly) - 1.11E-3 * sin(2*(anomaly+node));
    longitude = l;

    // Let's add nutation here also
    M1 = (124.90 - 1934.134 * calc->aux.t + 0.002063 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    M2 = (201.11 + 72001.5377 * calc->aux.t + 0.00057 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    d = - .0047785 * sin(M1) - .0003667 * sin(M2);
    longitude += d;

    // Get accurate Moon age
    Psin = 29.530588853;
    calc->moon_age = NormalizeRadians((longitude - calc->aux.slongitude) * snm_DEG_TO_RAD) * Psin / TWO_PI;

    // Now Moon parallax
    parallax = .950724 + .051818 * cos(anomaly) + .009531 * cos(2 * phase - anomaly);
    parallax += .007843 * cos(2 * phase) + .002824 * cos(2 * anomaly);
    parallax += 0.000857 * cos(2 * phase + anomaly) + E * .000533 * cos(2 * phase - calc->aux.sanomaly);
    parallax += E * .000401 * cos(2 * phase - anomaly - calc->aux.sanomaly) + E * .00032 * cos(anomaly - calc->aux.sanomaly) - .000271 * cos(phase);
    parallax += -E * .000264 * cos(calc->aux.sanomaly + anomaly) - .000198 * cos(2 * node - anomaly);
    parallax += 1.73E-4 * cos(3 * anomaly) + 1.67E-4 * cos(4*phase-anomaly);

    // So Moon distance in Earth radii is, more or less,
    distance = 1.0 / sin(parallax * snm_DEG_TO_RAD);

    // Ecliptic latitude with nodal phase (error<0.01 deg)
    l = 5.128189 * sin(node) + 0.280606 * sin(node + anomaly) + 0.277693 * sin(anomaly - node);
    l += .173238 * sin(2 * phase - node) + .055413 * sin(2 * phase + node - anomaly);
    l += .046272 * sin(2 * phase - node - anomaly) + .032573 * sin(2 * phase + node);
    l += .017198 * sin(2 * anomaly + node) + .009267 * sin(2 * phase + anomaly - node);
    l += .008823 * sin(2 * anomaly - node) + E * .008247 * sin(2 * phase - calc->aux.sanomaly - node) + .004323 * sin(2 * (phase - anomaly) - node);
    l += .0042 * sin(2 * phase + node + anomaly) + E * .003372 * sin(node - calc->aux.sanomaly - 2 * phase);
    l += E * 2.472E-3 * sin(2 * phase + node - calc->aux.sanomaly - anomaly);
    l += E * 2.222E-3 * sin(2 * phase + node - calc->aux.sanomaly);
    l += E * 2.072E-3 * sin(2 * phase - node - calc->aux.sanomaly - anomaly);
    latitude = l;

    params[0] = longitude;
    params[1] = latitude;
    params[2] = distance * EARTH_RADIUS / AU;
    params[3] = atan(1737.4 / (distance * EARTH_RADIUS));
}

void DoCalc(snm_Calculator *calc, double pos[], double params[])
{
    STATIC_OPT double t2, tmp, angle, M1, M2, d, cl;
    STATIC_OPT double x, y, z;
    STATIC_OPT double jd0, T0, secs, gmst, msday, lst, radiusAU;
    STATIC_OPT double xtopo, ytopo, ztopo;

    STATIC_OPT double ra, dec, dist, angh, sinlat, coslat, sindec, cosdec;
    STATIC_OPT double h, alt, azy, azx, azi;
    STATIC_OPT double r, refr;
    STATIC_OPT double celestialHoursToEarthTime;
    STATIC_OPT double transit_time1, transit_time2, transit_alt, transit_time;
    STATIC_OPT double jdToday;
    STATIC_OPT double transitToday2;
    STATIC_OPT double transit;
    STATIC_OPT double rise, set;
    STATIC_OPT double ang_hor;
    STATIC_OPT double rise_time1, set_time1;
    STATIC_OPT double rise_time2, set_time2;
    STATIC_OPT double rise_time, riseToday2;
    STATIC_OPT double set_time, setToday2;

    STATIC_OPT double correction[3];

    // Ecliptic to equatorial coordinates
    t2 = calc->aux.t / 100.0;
    tmp = t2 * (27.87 + t2 * (5.79 + t2 * 2.45));
    tmp = t2 * (-249.67 + t2 * (-39.05 + t2 * (7.12 + tmp)));
    tmp = t2 * (-1.55 + t2 * (1999.25 + t2 * (-51.38 + tmp)));
    tmp = (t2 * (-4680.93 + tmp)) / 3600.0;
    angle = (23.4392911111111 + tmp) * snm_DEG_TO_RAD; // obliquity

    // Add nutation in obliquity
    M1 = (124.90 - 1934.134 * calc->aux.t + 0.002063 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    M2 = (201.11 + 72001.5377 * calc->aux.t + 0.00057 * calc->aux.t * calc->aux.t) * snm_DEG_TO_RAD;
    d = .002558 * cos(M1) - .00015339 * cos(M2);
    angle += d * snm_DEG_TO_RAD;

    pos[0] *= snm_DEG_TO_RAD;
    pos[1] *= snm_DEG_TO_RAD;
    cl = cos(pos[1]);
    x = pos[2] * cos(pos[0]) * cl;
    y = pos[2] * sin(pos[0]) * cl;
    z = pos[2] * sin(pos[1]);
    tmp = y * cos(angle) - z * sin(angle);
    z = y * sin(angle) + z * cos(angle);
    y = tmp;

    // Obtain local apparent sidereal time
    jd0 = floor(calc->aux.jd_ut - 0.5) + 0.5;
    T0 = (jd0 - J2000) / JULIAN_DAYS_PER_CENTURY;
    secs = (calc->aux.jd_ut - jd0) * SECONDS_PER_DAY;
    gmst = (((((-6.2e-6 * T0) + 9.3104e-2) * T0) + 8640184.812866) * T0) + 24110.54841;
    msday = 1.0 + (((((-1.86e-5 * T0) + 0.186208) * T0) + 8640184.812866) / (SECONDS_PER_DAY * JULIAN_DAYS_PER_CENTURY));
    gmst = (gmst + msday * secs) * (15.0 / 3600.0) * snm_DEG_TO_RAD;
    lst = gmst + calc->longitude;

    // Obtain topocentric rectangular coordinates
    // Set radiusAU = 0 for geocentric calculations
    // (rise/set/transit will have no sense in this case)
    radiusAU = EARTH_RADIUS / AU;
    correction[0] = radiusAU * cos(calc->latitude) * cos(lst);
    correction[1] = radiusAU * cos(calc->latitude) * sin(lst);
    correction[2] = radiusAU * sin(calc->latitude);
    xtopo = x - correction[0];
    ytopo = y - correction[1];
    ztopo = z - correction[2];

    // Obtain topocentric equatorial coordinates
    ra = 0.0;
    dec = M_PI_2;

    if (ztopo < 0.0)
        dec = -dec;

    if (ytopo != 0.0 || xtopo != 0.0) {
        ra = atan2(ytopo, xtopo);
        dec = atan2(ztopo / sqrt(xtopo * xtopo +
                                             ytopo * ytopo), 1.0);
    }

    dist = sqrt(xtopo * xtopo +
                             ytopo * ytopo +
                             ztopo * ztopo);

    // Hour angle
    angh = lst - ra;

    // Obtain azimuth and geometric alt
    sinlat = sin(calc->latitude);
    coslat = cos(calc->latitude);
    sindec = sin(dec);
    cosdec = cos(dec);
    h = sinlat * sindec + coslat * cosdec * cos(angh);
    alt = asin(h);
    azy = sin(angh);
    azx = cos(angh) * sinlat - sindec * coslat / cosdec;
    azi = M_PI + atan2(azy, azx); // 0 = north

    // Get apparent elevation
    if (alt > -3 * snm_DEG_TO_RAD) {
        r = 0.016667 * snm_DEG_TO_RAD * fabs(tan(M_PI_2 - (alt * snm_RAD_TO_DEG +  7.31 / (alt * snm_RAD_TO_DEG + 4.4)) * snm_DEG_TO_RAD));
        refr = r * ( 0.28 * 1010 / (10 + 273.0)); // Assuming pressure of 1010 mb and T = 10 C
        alt = fmin(alt + refr, M_PI_2); // This is not accurate, but acceptable
    }

    switch (calc->twilight) {
    case HORIZON_34arcmin:
        // Rise, set, transit times, taking into account Sun/Moon angular radius (pos[3]).
        // The 34' factor is the standard refraction at horizon.
        // Removing angular radius will do calculations for the center of the disk instead
        // of the upper limb.
        tmp = -(34.0 / 60.0) * snm_DEG_TO_RAD - pos[3];
        break;
    case TWILIGHT_CIVIL:
        tmp = -6 * snm_DEG_TO_RAD;
        break;
    case TWILIGHT_NAUTICAL:
        tmp = -12 * snm_DEG_TO_RAD;
        break;
    case TWILIGHT_ASTRONOMICAL:
        tmp = -18 * snm_DEG_TO_RAD;
        break;
    }

    // Compute cosine of hour angle
    tmp = (sin(tmp) - sin(calc->latitude) * sin(dec)) / (cos(calc->latitude) * cos(dec));
    celestialHoursToEarthTime = RAD_TO_DAY / SIDEREAL_DAY_LENGTH;

    // Make calculations for the meridian
    transit_time1 = celestialHoursToEarthTime * NormalizeRadians(ra - lst);
    transit_time2 = celestialHoursToEarthTime * (NormalizeRadians(ra - lst) - TWO_PI);
    transit_alt = asin(sin(dec) * sin(calc->latitude) + cos(dec) * cos(calc->latitude));

    if (transit_alt > -3 * snm_DEG_TO_RAD) {
        r = 0.016667 * snm_DEG_TO_RAD * fabs(tan(M_PI_2 - (transit_alt * snm_RAD_TO_DEG +  7.31 / (transit_alt * snm_RAD_TO_DEG + 4.4)) * snm_DEG_TO_RAD));
        refr = r * ( 0.28 * 1010 / (10 + 273.0)); // Assuming pressure of 1010 mb and T = 10 C
        transit_alt = fmin(transit_alt + refr, M_PI_2); // This is not accurate, but acceptable
    }

    // Obtain the current event in time
    transit_time = transit_time1;
    jdToday = floor(calc->aux.jd_ut - 0.5) + 0.5;
    transitToday2 = floor(calc->aux.jd_ut + transit_time2 - 0.5) + 0.5;
    // Obtain the transit time. Preference should be given to the closest event
    // in time to the current calculation time
    if (jdToday == transitToday2 && fabs(transit_time2) < fabs(transit_time1)) transit_time = transit_time2;
    transit = calc->aux.jd_ut + transit_time;

    // Make calculations for rise and set
    rise = -1;
    set = -1;

    if (fabs(tmp) <= 1.0) {
        ang_hor = fabs(acos(tmp));
        rise_time1 = celestialHoursToEarthTime * NormalizeRadians(ra - ang_hor - lst);
        set_time1 = celestialHoursToEarthTime * NormalizeRadians(ra + ang_hor - lst);
        rise_time2 = celestialHoursToEarthTime * (NormalizeRadians(ra - ang_hor - lst) - TWO_PI);
        set_time2 = celestialHoursToEarthTime * (NormalizeRadians(ra + ang_hor - lst) - TWO_PI);

        // Obtain the current events in time. Preference should be given to the closest event
        // in time to the current calculation time (so that iteration in other method will converge)
        rise_time = rise_time1;
        riseToday2 = floor(calc->aux.jd_ut + rise_time2 - 0.5) + 0.5;

        if (jdToday == riseToday2 && fabs(rise_time2) < fabs(rise_time1))
            rise_time = rise_time2;

        set_time = set_time1;
        setToday2 = floor(calc->aux.jd_ut + set_time2 - 0.5) + 0.5;

        if (jdToday == setToday2 && fabs(set_time2) < fabs(set_time1))
            set_time = set_time2;

        rise = calc->aux.jd_ut + rise_time;
        set = calc->aux.jd_ut + set_time;
    }

    params[0] = azi;
    params[1] = alt;
    params[2] = rise;
    params[3] = set;
    params[4] = transit;
    params[5] = transit_alt;
    params[6] = ra;
    params[7] = dec;
    params[8] = dist;
}

double ObtainAccurateRiseSetTransit(snm_Calculator *calc, double riseSetJD, int index, int niter, int sun)
{
    STATIC_OPT double step = -1;
    STATIC_OPT double out[9];
    STATIC_OPT double sun_params[4];
    STATIC_OPT double moon_params[4];

    step = -1;

    for (int i = 0; i < niter; i++) {
        if (riseSetJD == -1)
            return riseSetJD; // -1 means no rise/set from that location

        SetUTDate(calc, riseSetJD);
        

        if (sun) {
            GetSun(calc, sun_params);
            DoCalc(calc, sun_params, out);
        } else {
            GetMoon(calc, moon_params);
            DoCalc(calc, moon_params, out);
        }

        step = fabs(riseSetJD - out[index]);
        riseSetJD = out[index];
    }

    if (step > 1.0 / SECONDS_PER_DAY)
        return -1; // did not converge => without rise/set/transit in this date

    return riseSetJD;
}

//////////////////////////////////////////////////////////////////////////////////


void snm_CalculatorCreate(snm_Calculator *calc)
{
    memset(calc, 0, sizeof(snm_Calculator));
}

void snm_CalculatorSetTime(snm_Calculator *calc, int day, int month, int year)
{
    calc->day = day;
    calc->month = month;
    calc->year = year;
}

void snm_CalculatorSetDate(snm_Calculator *calc, int hour, int minute, int second)
{
    calc->hour = hour;
    calc->minute = minute;
    calc->second = second;
}

void snm_CalculatorSetPoint(snm_Calculator *calc, double latitude, double longitude)
{
    calc->latitude = latitude;
    calc->longitude = longitude;
}

void snm_CalculatorSetTwilight(snm_Calculator *calc, 
                               TWILIGHT twilight)
{
    calc->twilight = twilight;
}

void snm_CalculatorCalc(snm_Calculator *calc)
{
    double sun_params[4];
    double out[9];
    double jd, sa, sl, ma;
    int niter = 3; //Number of iterations to get accurate rise/set/transit times

    SetTimes(calc);

    jd = calc->aux.jd_ut;

    // First the Sun
    GetSun(calc, sun_params);
    DoCalc(calc, sun_params, out);

    calc->sun_az = out[0];
    calc->sun_elev = out[1];
    calc->sun_rise = out[2];
    calc->sun_set = out[3];
    calc->sun_transit = out[4];
    calc->sun_transit_elev = out[5];
    calc->sun_dist = out[8];

    sa = calc->aux.sanomaly;
    sl = calc->aux.slongitude;

    calc->sun_rise = ObtainAccurateRiseSetTransit(calc, calc->sun_rise, 2, niter, TRUE);
    calc->sun_set = ObtainAccurateRiseSetTransit(calc, calc->sun_set, 3, niter, TRUE);
    calc->sun_transit = ObtainAccurateRiseSetTransit(calc, calc->sun_transit, 4, niter, TRUE);

    if (calc->sun_transit == -1) {
        calc->sun_transit_elev = 0;
    } else {
        // Update Sun's maximum elevation
        SetUTDate(calc, calc->sun_transit);
        GetSun(calc, sun_params);
        DoCalc(calc, sun_params, out);
        calc->sun_transit_elev = out[5];
    }

    // Now Moon
    SetUTDate(calc, jd);
    calc->aux.sanomaly = sa;
    calc->aux.slongitude = sl;
    GetMoon(calc, sun_params);
    DoCalc(calc, sun_params, out);

    calc->moon_az = out[0];
    calc->moon_elev = out[1];
    calc->moon_rise = out[2];
    calc->moon_set = out[3];
    calc->moon_transit = out[4];
    calc->moon_transit_elev = out[5];
    calc->moon_dist = out[8];
    ma = calc->moon_age;

    niter = 5; // Number of iterations to get accurate rise/set/transit times
    calc->moon_rise = ObtainAccurateRiseSetTransit(
                calc, calc->moon_rise, 2, niter, FALSE);
    calc->moon_set = ObtainAccurateRiseSetTransit(
                calc, calc->moon_set, 3, niter, FALSE);
    calc->moon_transit = ObtainAccurateRiseSetTransit(
                calc, calc->moon_transit, 4, niter, FALSE);

    if (calc->moon_transit == -1) {
        calc->moon_transit_elev = 0;
    } else {
        // Update Moon's maximum elevation
        SetUTDate(calc, calc->moon_transit);
        // double tmp[4];
        // GetSun(calc, tmp);
        GetMoon(calc, sun_params);
        DoCalc(calc, sun_params, out);
        calc->moon_transit_elev = out[5];
    }

    SetUTDate(calc, jd);
    calc->aux.sanomaly = sa;
    calc->aux.slongitude = sl;
    calc->moon_age = ma;
}

void GetDate(double jd, double params[]) 
{
    STATIC_OPT double Z, F, A, B;
    STATIC_OPT int a, C, D, E;
    STATIC_OPT double exactDay;
    STATIC_OPT int day, month, year;
    STATIC_OPT double h, m;
    STATIC_OPT int hour, minute, second;

    if (jd < 2299160.0 && jd >= 2299150.0) {
        fprintf(stderr, 
                "Invalid julian day %f. "
                "This date does not exist.\n", 
                jd);
        return;
    }

    // The conversion formulas are from Meeus,
    // Chapter 7.
    Z = floor(jd + 0.5);
    F = jd + 0.5 - Z;
    A = Z;

    if (Z >= 2299161) {
        a = (int) ((Z - 1867216.25) / 36524.25);
        A += 1 + a - a / 4;
    }

    B = A + 1524;
    C = (int) ((B - 122.1) / 365.25);
    D = (int) (C * 365.25);
    E = (int) ((B - D) / 30.6001);

    exactDay = F + B - D - (int) (30.6001 * E);
    day = (int) exactDay;
    month = (E < 14) ? E - 1 : E - 13;
    year = C - 4715;

    if (month > 2)
        year--;

    h = ((exactDay - day) * SECONDS_PER_DAY) / 3600.0;

    hour = (int) h;
    m = (h - hour) * 60.0;
    minute = (int) m;
    second = (int) ((m - minute) * 60.0);

    params[0] = year;
    params[1] = month;
    params[2] = day;
    params[3] = hour;
    params[4] = minute;
    params[5] = second;
}

struct tm snm_CalculatorGetDateAsTm(double jd,
                                    double tz)
{
    STATIC_OPT struct tm timestamp = {0};
    STATIC_OPT double date[6];
    STATIC_OPT int year, month, day, hour, minute, second;
    
    if (jd == -1) {
        fprintf(stderr, 
                "Invalid julian day %f. "
                "This date does not exist.\n", 
                jd);
        return timestamp;
    }

    memset(date, 0, sizeof(date));
    GetDate(jd, date);

    year = date[0];
    month = date[1];
    day = date[2];
    hour = date[3];
    minute = date[4];
    second = date[5];

    timestamp.tm_year = year - 1900;
    timestamp.tm_mon = month - 1;
    timestamp.tm_mday = day;
    timestamp.tm_hour = hour;
    timestamp.tm_min = minute + tz * 60.;
    timestamp.tm_sec = second;

    mktime(&timestamp);

    return timestamp;
}
