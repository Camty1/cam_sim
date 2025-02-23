#ifndef TIME_H
#define TIME_H
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>

#define SECONDS_PER_DAY 86400

namespace CamSim::Time {

class Timestamp
{
public:
    static Timestamp now()
    {
        return Timestamp();
    }

    static Timestamp from_posix_timestamp(const ulong seconds, const ulong nanoseconds = 0)
    {
        return Timestamp(seconds, nanoseconds);
    }

    static Timestamp from_posix_timestamp(const double posix_timestamp)
    {
        const double seconds = std::floor(posix_timestamp);
        const double nanoseconds = 1e9 * (posix_timestamp - seconds);

        return Timestamp((long)seconds, (long)nanoseconds);
    }

    static Timestamp from_jd_utc(const double jd_utc)
    {
        const double posix_timestamp = (jd_utc - jd_offset) * SECONDS_PER_DAY;

        return from_posix_timestamp(posix_timestamp);
    }

    static Timestamp from_jd_gps(const double jd_gps)
    {
        const double gps_timestamp = (jd_gps - jd_offset) * SECONDS_PER_DAY;
        const double tai_timestamp = gps_timestamp + gps_epoch_offset + gps_tai_offset;

        // This isn't exactly correct when you are close to leap seconds, but it is close enough.
        const double posix_timestamp = tai_timestamp - get_utc_leap_seconds((ulong)tai_timestamp);

        return from_posix_timestamp(posix_timestamp);
    }

    static Timestamp from_decimal_year(const double decimal_year)
    {
        int year = std::floor(decimal_year);
        double decimal = decimal_year - year;
        int index = year - 1970;

        double start_timestamp = std::get<1>(year_timestamps_table[index]);
        double end_timestamp = std::get<2>(year_timestamps_table[index]);

        return from_posix_timestamp(start_timestamp + (end_timestamp - start_timestamp) * decimal);
    }

    double get_utc_timestamp() const
    {
        return posix_timestamp.tv_sec + posix_timestamp.tv_nsec / 1e9;
    }

    double get_gps_timestamp() const
    {
        return get_tai_timestamp() - gps_epoch_offset - gps_tai_offset;
    }

    double get_jd_utc() const
    {
        return get_utc_timestamp() / (double)SECONDS_PER_DAY + jd_offset;
    }

    double get_jd_gps() const
    {
        return get_gps_timestamp() / (double)SECONDS_PER_DAY + jd_offset;
    }

    double get_decimal_year() const
    {
        double timestamp = get_utc_timestamp();
        for (auto& [year, start_timestamp, end_timestamp] : year_timestamps_table)
        {
            if (start_timestamp <= timestamp && timestamp < end_timestamp)
            {
                return year + (timestamp - start_timestamp) / (end_timestamp - start_timestamp);
            }
        }
        throw std::range_error("Timestamp year is out of range [1970, 2030)");
    }

private:
    Timestamp()
    {
        std::timespec_get(&posix_timestamp, TIME_UTC);
    }

    Timestamp(ulong seconds, ulong nanoseconds)
    {
        posix_timestamp.tv_sec = seconds;
        posix_timestamp.tv_nsec = nanoseconds;
    }

    double get_tai_timestamp() const
    {
        return get_utc_timestamp() + get_utc_leap_seconds(posix_timestamp.tv_sec);
    }

    static double get_utc_leap_seconds(ulong seconds);

    std::timespec posix_timestamp;
    static constexpr double gps_epoch_offset = 315964800.0;
    static constexpr double gps_tai_offset = 19.0;
    static constexpr double jd_offset = 2440587.5;

    static constexpr std::array<std::tuple<ulong, double>, 29> leap_seconds_table = {{
        {0, 4.2131700},     {63072000, 10.0},   {78796800, 11.0},   {94694400, 12.0},
        {126230400, 13.0},  {157766400, 14.0},  {189302400, 15.0},  {220924800, 16.0},
        {252460800, 17.0},  {283996800, 18.0},  {315532800, 19.0},  {362793600, 20.0},
        {394329600, 21.0},  {425865600, 22.0},  {489024000, 23.0},  {567993600, 24.0},
        {631152000, 25.0},  {662688000, 26.0},  {709948800, 27.0},  {741484800, 28.0},
        {773020800, 29.0},  {820454400, 30.0},  {867715200, 31.0},  {915148800, 32.0},
        {1136073600, 33.0}, {1230768000, 34.0}, {1341100800, 35.0}, {1435708800, 36.0},
        {1483228800, 37.0},
    }};

    static constexpr std::array<std::tuple<double, double, double>, 60> year_timestamps_table = {{
        {1970.0, 0.0, 31536000.0},
        {1971.0, 31536000.0, 63072000.0},
        {1972.0, 63072000.0, 94694400.0},
        {1973.0, 94694400.0, 126230400.0},
        {1974.0, 126230400.0, 157766400.0},
        {1975.0, 157766400.0, 189302400.0},
        {1976.0, 189302400.0, 220924800.0},
        {1977.0, 220924800.0, 252460800.0},
        {1978.0, 252460800.0, 283996800.0},
        {1979.0, 283996800.0, 315532800.0},
        {1980.0, 315532800.0, 347155200.0},
        {1981.0, 347155200.0, 378691200.0},
        {1982.0, 378691200.0, 410227200.0},
        {1983.0, 410227200.0, 441763200.0},
        {1984.0, 441763200.0, 473385600.0},
        {1985.0, 473385600.0, 504921600.0},
        {1986.0, 504921600.0, 536457600.0},
        {1987.0, 536457600.0, 567993600.0},
        {1988.0, 567993600.0, 599616000.0},
        {1989.0, 599616000.0, 631152000.0},
        {1990.0, 631152000.0, 662688000.0},
        {1991.0, 662688000.0, 694224000.0},
        {1992.0, 694224000.0, 725846400.0},
        {1993.0, 725846400.0, 757382400.0},
        {1994.0, 757382400.0, 788918400.0},
        {1995.0, 788918400.0, 820454400.0},
        {1996.0, 820454400.0, 852076800.0},
        {1997.0, 852076800.0, 883612800.0},
        {1998.0, 883612800.0, 915148800.0},
        {1999.0, 915148800.0, 946684800.0},
        {2000.0, 946684800.0, 978307200.0},
        {2001.0, 978307200.0, 1009843200.0},
        {2002.0, 1009843200.0, 1041379200.0},
        {2003.0, 1041379200.0, 1072915200.0},
        {2004.0, 1072915200.0, 1104537600.0},
        {2005.0, 1104537600.0, 1136073600.0},
        {2006.0, 1136073600.0, 1167609600.0},
        {2007.0, 1167609600.0, 1199145600.0},
        {2008.0, 1199145600.0, 1230768000.0},
        {2009.0, 1230768000.0, 1262304000.0},
        {2010.0, 1262304000.0, 1293840000.0},
        {2011.0, 1293840000.0, 1325376000.0},
        {2012.0, 1325376000.0, 1356998400.0},
        {2013.0, 1356998400.0, 1388534400.0},
        {2014.0, 1388534400.0, 1420070400.0},
        {2015.0, 1420070400.0, 1451606400.0},
        {2016.0, 1451606400.0, 1483228800.0},
        {2017.0, 1483228800.0, 1514764800.0},
        {2018.0, 1514764800.0, 1546300800.0},
        {2019.0, 1546300800.0, 1577836800.0},
        {2020.0, 1577836800.0, 1609459200.0},
        {2021.0, 1609459200.0, 1640995200.0},
        {2022.0, 1640995200.0, 1672531200.0},
        {2023.0, 1672531200.0, 1704067200.0},
        {2024.0, 1704067200.0, 1735689600.0},
        {2025.0, 1735689600.0, 1767225600.0},
        {2026.0, 1767225600.0, 1798761600.0},
        {2027.0, 1798761600.0, 1830297600.0},
        {2028.0, 1830297600.0, 1861920000.0},
        {2029.0, 1861920000.0, 1893456000.0},
    }};
};

}

#endif
