#ifndef UNIT_CONVERSION_H
#define UNIT_CONVERSION_H

constexpr double UM_PER_PIXEL = 5.2f;

inline double pxToUm(double px)
{
    return px * UM_PER_PIXEL;
}

inline double umToPx(double um)
{
    return um / UM_PER_PIXEL;
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
    return px * (UM_PER_PIXEL / 1000.0);
}

inline double mmToPx(double mm)
{
    return mm * 1000.0 / UM_PER_PIXEL;
}

#endif  // UNIT_CONVERSION_H
