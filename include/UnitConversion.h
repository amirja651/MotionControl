#ifndef UNIT_CONVERSION_H
#define UNIT_CONVERSION_H

constexpr double PIXELS_PER_UM = 5.2;

inline double pxToUm(double px)
{
    return px / PIXELS_PER_UM;
}

inline double umToPx(double um)
{
    return um * PIXELS_PER_UM;
}

inline double umToMm(double um)
{
    return um / 1000.0;
}

inline double mmToUm(double mm)
{
    return mm * 1000.0;
}

inline double pxToMm(double px)
{
    return px / (PIXELS_PER_UM * 1000.0);
}

inline double mmToPx(double mm)
{
    return mm * (PIXELS_PER_UM * 1000.0);
}

#endif  // UNIT_CONVERSION_H
