#include "worldtoscreenconverter.h"

int toInt(double d) { return int(round(d)); }

WorldToScreenConverter::WorldToScreenConverter(State const& state, QRect screen) noexcept
    : state(state), screen(screen)
{
    double worldWidth = state.worldEnd.x - state.worldOrigin.x;
    double worldHeight = state.worldEnd.y - state.worldOrigin.y;
    scale = std::min(screen.width() / worldWidth, screen.height()/worldHeight);
}

double WorldToScreenConverter::x(double x) const noexcept {
    return ( x - state.worldOrigin.x ) * scale;
}

double WorldToScreenConverter::y(double y) const noexcept {
    return ( y - state.worldOrigin.y ) * scale;
}

double WorldToScreenConverter::r(double r) const noexcept {
    return r * scale;
}

QRect WorldToScreenConverter::circle(Position pos, double radius) const noexcept {
    double R = r(radius);
    return QRect(toInt(x(pos.x)-R),toInt(y(pos.y)-R),toInt(2*R),toInt(2*R));
}

QRect WorldToScreenConverter::world() const noexcept {
    return QRect(toInt(x(state.worldOrigin.x)),
                 toInt(y(state.worldOrigin.y)),
                 toInt(r(state.worldEnd.x - state.worldOrigin.x)),
                 toInt(r(state.worldEnd.y - state.worldOrigin.y)));
}
